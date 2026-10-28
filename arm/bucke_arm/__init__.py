"""Buck-E arm preparation; no hardware initialization on import."""

# Marks bucke_arm as a Python package. Keep startup free of robot side effects.
# CLI behavior lives in __main__.py; calculations and hardware adapters are
# imported explicitly by their callers rather than initialized here.
