"""Conversion core for external aerodynamic solver output.

``aero_convert.units`` holds the unit walls, the CG moment transfer and the
sign normalisation; ``aero_convert.morelli`` maps a normalised derivative set to
the 19 Morelli arrays.  Solver adapters arrive later in ``aero_convert.solvers``
and are deliberately not imported here, so this package stays usable as pure
maths with no solver on the machine.
"""