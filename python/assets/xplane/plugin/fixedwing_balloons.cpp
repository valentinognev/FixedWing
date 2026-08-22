/*
 * fixedwing_balloons — X-Plane 12 plugin for FixedWing balloon race.
 * UDP 49091 JSON: place / clear / pose_query / sitl_connect
 */
#include <XPLMDataAccess.h>
#include <XPLMDisplay.h>
#include <XPLMGraphics.h>
#include <XPLMInstance.h>
#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <XPLMScenery.h>
#include <XPLMUtilities.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace {

constexpr int kPort = 49091;
constexpr int kMaxBalloons = 16;

struct Balloon {
	std::string name;
	double lat = 0;
	double lon = 0;
	double alt_msl_m = 0;
	float diameter_m = 2.f;
	float rgb[3] = {1.f, 0.f, 0.f};
	XPLMInstanceRef inst = nullptr;
};

int g_sock = -1;
std::mutex g_mu;
std::map<std::string, Balloon> g_balloons;
XPLMObjectRef g_obj = nullptr;
XPLMDataRef g_lat = nullptr;
XPLMDataRef g_lon = nullptr;
XPLMDataRef g_alt = nullptr;
XPLMDataRef g_roll = nullptr;
XPLMDataRef g_pitch = nullptr;
XPLMDataRef g_psi = nullptr;
std::string g_plugin_path;

std::string trim(const std::string& s) {
	size_t a = s.find_first_not_of(" \t\r\n");
	if (a == std::string::npos) return "";
	size_t b = s.find_last_not_of(" \t\r\n");
	return s.substr(a, b - a + 1);
}

// Minimal JSON field extractors (no full parser).
std::string json_str(const std::string& j, const char* key) {
	std::string pat = std::string("\"") + key + "\"";
	size_t p = j.find(pat);
	if (p == std::string::npos) return "";
	p = j.find(':', p);
	if (p == std::string::npos) return "";
	p = j.find('"', p);
	if (p == std::string::npos) return "";
	size_t e = j.find('"', p + 1);
	if (e == std::string::npos) return "";
	return j.substr(p + 1, e - p - 1);
}

double json_num(const std::string& j, const char* key, double def = 0) {
	std::string pat = std::string("\"") + key + "\"";
	size_t p = j.find(pat);
	if (p == std::string::npos) return def;
	p = j.find(':', p);
	if (p == std::string::npos) return def;
	return std::strtod(j.c_str() + p + 1, nullptr);
}

void json_rgb(const std::string& j, float out[3]) {
	out[0] = out[1] = out[2] = 1.f;
	size_t p = j.find("\"rgb\"");
	if (p == std::string::npos) return;
	p = j.find('[', p);
	if (p == std::string::npos) return;
	int r = 255, g = 0, b = 0;
	std::sscanf(j.c_str() + p, "[%d,%d,%d]", &r, &g, &b);
	out[0] = r / 255.f;
	out[1] = g / 255.f;
	out[2] = b / 255.f;
}

void destroy_instance(Balloon& b) {
	if (b.inst) {
		XPLMDestroyInstance(b.inst);
		b.inst = nullptr;
	}
}

void clear_all() {
	for (auto& kv : g_balloons) destroy_instance(kv.second);
	g_balloons.clear();
}

bool ensure_obj() {
	if (g_obj) return true;
	std::string path = g_plugin_path + "balloon_sphere.obj";
	g_obj = XPLMLoadObject(path.c_str());
	if (!g_obj) {
		XPLMDebugString("fixedwing_balloons: failed to load balloon_sphere.obj\n");
		return false;
	}
	return true;
}

void set_instance_pose(Balloon& b) {
	if (!b.inst) return;
	double x = 0, y = 0, z = 0;
	XPLMWorldToLocal(b.lat, b.lon, b.alt_msl_m, &x, &y, &z);
	XPLMDrawInfo_t info{};
	info.structSize = sizeof(info);
	info.x = (float)x;
	info.y = (float)y;
	info.z = (float)z;
	info.pitch = 0.f;
	info.heading = 0.f;
	info.roll = 0.f;
	// Pre-11.50 requires a non-null data pointer even with zero datarefs.
	float dummy = 0.f;
	XPLMInstanceSetPosition(b.inst, &info, &dummy);
}

void refresh_all_poses() {
	for (auto& kv : g_balloons) set_instance_pose(kv.second);
}

void place_balloon(const std::string& name, double lat, double lon, double alt,
                   float diam, float rgb[3]) {
	if ((int)g_balloons.size() >= kMaxBalloons && !g_balloons.count(name)) {
		return;
	}
	Balloon& b = g_balloons[name];
	destroy_instance(b);
	b.name = name;
	b.lat = lat;
	b.lon = lon;
	b.alt_msl_m = alt;
	b.diameter_m = diam > 0.1f ? diam : 2.f;
	b.rgb[0] = rgb[0];
	b.rgb[1] = rgb[1];
	b.rgb[2] = rgb[2];
	if (!ensure_obj()) return;
	static const char* kNoDrefs[] = {nullptr};
	b.inst = XPLMCreateInstance(g_obj, kNoDrefs);
	if (!b.inst) return;
	set_instance_pose(b);
	(void)rgb;  // colour reserved for future tinted OBJ / dataref
}

std::string handle_cmd(const std::string& raw) {
	std::string j = trim(raw);
	std::string cmd = json_str(j, "cmd");
	if (cmd.empty()) {
		// tolerate {"cmd":place} without quotes on value
		if (j.find("\"cmd\":\"place\"") != std::string::npos || j.find("\"cmd\": \"place\"") != std::string::npos)
			cmd = "place";
		else if (j.find("clear") != std::string::npos) cmd = "clear";
		else if (j.find("pose_query") != std::string::npos) cmd = "pose_query";
		else if (j.find("sitl_connect") != std::string::npos) cmd = "sitl_connect";
	}

	if (cmd == "clear") {
		std::lock_guard<std::mutex> lock(g_mu);
		clear_all();
		return "{\"ok\":true,\"cmd\":\"clear\"}";
	}
	if (cmd == "sitl_connect") {
		XPLMCommandRef c = XPLMFindCommand("px4xplane/toggleEnable");
		if (c) XPLMCommandOnce(c);
		return "{\"ok\":true,\"cmd\":\"sitl_connect\"}";
	}
	if (cmd == "pose_query") {
		double lat = g_lat ? XPLMGetDatad(g_lat) : 0;
		double lon = g_lon ? XPLMGetDatad(g_lon) : 0;
		double elev = g_alt ? XPLMGetDatad(g_alt) : 0;  // meters MSL
		float roll = g_roll ? XPLMGetDataf(g_roll) : 0;
		float pitch = g_pitch ? XPLMGetDataf(g_pitch) : 0;
		float psi = g_psi ? XPLMGetDataf(g_psi) : 0;
		char buf[256];
		std::snprintf(buf, sizeof(buf),
			"{\"ok\":true,\"cmd\":\"pose_query\",\"lat\":%.8f,\"lon\":%.8f,"
			"\"alt_msl_m\":%.3f,\"roll_deg\":%.3f,\"pitch_deg\":%.3f,\"heading_deg\":%.3f}",
			lat, lon, elev, roll, pitch, psi);
		return buf;
	}
	if (cmd == "place") {
		std::string name = json_str(j, "name");
		if (name.empty()) name = "balloon";
		double lat = json_num(j, "lat");
		double lon = json_num(j, "lon");
		double alt = json_num(j, "alt_msl_m");
		float diam = (float)json_num(j, "diameter_m", 2.0);
		float rgb[3];
		json_rgb(j, rgb);
		std::lock_guard<std::mutex> lock(g_mu);
		place_balloon(name, lat, lon, alt, diam, rgb);
		return "{\"ok\":true,\"cmd\":\"place\",\"name\":\"" + name + "\"}";
	}
	return "{\"ok\":false,\"error\":\"unknown_cmd\"}";
}

void drain_udp() {
	if (g_sock < 0) return;
	char buf[2048];
	sockaddr_in peer{};
	socklen_t plen = sizeof(peer);
	for (;;) {
		ssize_t n = recvfrom(g_sock, buf, sizeof(buf) - 1, MSG_DONTWAIT,
			reinterpret_cast<sockaddr*>(&peer), &plen);
		if (n <= 0) break;
		buf[n] = 0;
		std::string reply = handle_cmd(buf);
		sendto(g_sock, reply.data(), reply.size(), 0,
			reinterpret_cast<sockaddr*>(&peer), plen);
	}
}

float flight_loop(float /*elapsed*/, float /*since*/, int /*counter*/, void* /*ref*/) {
	drain_udp();
	{
		std::lock_guard<std::mutex> lock(g_mu);
		refresh_all_poses();
	}
	return -1.f;  // every frame
}

}  // namespace

PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc) {
	std::strcpy(outName, "fixedwing_balloons");
	std::strcpy(outSig, "fixedwing.balloons");
	std::strcpy(outDesc, "UDP balloon place/pose for FixedWing race");

	char path[512];
	XPLMGetPluginInfo(XPLMGetMyID(), nullptr, path, nullptr, nullptr);
	// path ends with .../64/lin.xpl → parent of 64 is plugin root
	std::string p(path);
	size_t slash = p.find_last_of('/');
	if (slash != std::string::npos) p.resize(slash);
	slash = p.find_last_of('/');
	if (slash != std::string::npos && p.substr(slash + 1) == "64") {
		p.resize(slash + 1);
	} else if (!p.empty() && p.back() != '/') {
		p.push_back('/');
	}
	g_plugin_path = p;

	g_lat = XPLMFindDataRef("sim/flightmodel/position/latitude");
	g_lon = XPLMFindDataRef("sim/flightmodel/position/longitude");
	g_alt = XPLMFindDataRef("sim/flightmodel/position/elevation");
	g_roll = XPLMFindDataRef("sim/flightmodel/position/phi");
	g_pitch = XPLMFindDataRef("sim/flightmodel/position/theta");
	g_psi = XPLMFindDataRef("sim/flightmodel/position/psi");

	g_sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (g_sock >= 0) {
		int yes = 1;
		setsockopt(g_sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
		sockaddr_in addr{};
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = htonl(INADDR_ANY);
		addr.sin_port = htons(kPort);
		if (bind(g_sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
			XPLMDebugString("fixedwing_balloons: bind 49091 failed\n");
			close(g_sock);
			g_sock = -1;
		}
	}

	XPLMRegisterFlightLoopCallback(flight_loop, -1.f, nullptr);
	return 1;
}

PLUGIN_API void XPluginStop(void) {
	XPLMUnregisterFlightLoopCallback(flight_loop, nullptr);
	clear_all();
	if (g_obj) {
		XPLMUnloadObject(g_obj);
		g_obj = nullptr;
	}
	if (g_sock >= 0) {
		close(g_sock);
		g_sock = -1;
	}
}

PLUGIN_API void XPluginDisable(void) {}
PLUGIN_API int XPluginEnable(void) { return 1; }
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID /*from*/, int /*msg*/, void* /*param*/) {}
