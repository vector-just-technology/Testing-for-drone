# Implementation for variables #
try: 
    import os
    import variables
    from variables import *
    import automations
    from automations import *
    import terminal_styling
    from terminal_styling import *
    import autopilot
    from autopilot import *
    import firstpersonviewcontroller
    from firstpersonviewcontroller import *
    import dependicies
    from dependicies import *
    import newflightcontroller
    from newflightcontroller import *
except ImportError as missing:
    print("An error happened. Maybe a file is missing?", missing)
print("Autopilot Ready -- All files are present and ready")
print("NOTICE: YOU WILL NEED TO EXTRACT CODE FROM OTHER FILES IN ORDER TO WORK")
