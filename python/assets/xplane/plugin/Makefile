# Build fixedwing_balloons for X-Plane 12 Linux (lin.xpl).
# Headers from Laminar SDK zip; XPLM symbols resolved by the sim at load time.
#
#   make SDK_DIR=/tmp/xplane_sdk OUTDIR=/tmp/fw_balloons_build

CXX ?= g++
SDK_DIR ?= /tmp/xplane_sdk
OUTDIR ?= .
SDK_ZIP_URL ?= https://developer.x-plane.com/wp-content/plugins/code-sample-generation/sdk_zip_files/XPSDK411.zip

CXXFLAGS += -std=c++17 -fPIC -O2 -Wall -Wextra \
	-DXPLM200 -DXPLM210 -DXPLM300 -DXPLM301 -DXPLM400 -DXPLM303 -DAPL=0 -DIBM=0 -DLIN=1 \
	-I$(SDK_DIR)/CHeaders/XPLM -I$(SDK_DIR)/CHeaders/Widgets

# Do not link libXPLM — X-Plane provides symbols when it loads the plugin.
LDFLAGS += -shared -fvisibility=hidden -Wl,--allow-shlib-undefined

.PHONY: all clean ensure_sdk

all: ensure_sdk $(OUTDIR)/lin.xpl

ensure_sdk:
	@if [ ! -f "$(SDK_DIR)/CHeaders/XPLM/XPLMPlugin.h" ]; then \
		echo "Downloading X-Plane SDK → $(SDK_DIR)"; \
		mkdir -p "$(SDK_DIR)"; \
		TMP=$$(mktemp -d); \
		curl -fsSL -o "$$TMP/sdk.zip" "$(SDK_ZIP_URL)"; \
		unzip -qo "$$TMP/sdk.zip" -d "$$TMP/out"; \
		SRC=$$(find "$$TMP/out" -type d -name 'CHeaders' | head -1 | xargs dirname); \
		cp -a "$$SRC/." "$(SDK_DIR)/"; \
		rm -rf "$$TMP"; \
	fi

$(OUTDIR)/lin.xpl: fixedwing_balloons.cpp ensure_sdk
	mkdir -p "$(OUTDIR)"
	$(CXX) $(CXXFLAGS) -o "$@" fixedwing_balloons.cpp $(LDFLAGS)

clean:
	rm -f "$(OUTDIR)/lin.xpl"
