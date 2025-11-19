#!/usr/bin/env python3

import rerun as rr
import rerun.blueprint as rrb
import sys
import os

def create_blueprints():
    """Create and return the blueprints for the rerun viewer"""
    
    # Default blueprint
    contents = ['+ /image/**']
    topleft = rrb.Spatial2DView(name="Vision", contents=contents)
    
    contents = ['+ /**']
    topright = rrb.TextLogView(name="Text", contents=contents)
    
    contents = ['+ /soccerfield/**', '- /soccerfield/team_com/**', '- /soccerfield/localization/hypotheses/**']
    bottomleft = rrb.Spatial2DView(name="Soccer Field", contents=contents, background=[34, 139, 36])
    
    contents = ['+ /relative/**']
    bottomright = rrb.Spatial2DView(name="Relative", contents=contents)
    
    default = rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(
                topleft,
                topright,
            ),
            rrb.Horizontal(
                bottomleft,
                bottomright,
            ),
        ),
        collapse_panels=True,
    )
    
    # Vision blueprint
    vision = rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="vision", name="vision"),
            rrb.Spatial2DView(origin="vision", name="vision2")
        ),
        collapse_panels=True,
    )
    
    return default, vision

def start_grpc_viewer():
    """Start rerun viewer expecting GRPC stream"""
    
    APP_ID = "fw_salvador_online"
    
    # Initialize rerun for GRPC streaming
    rr.init(APP_ID, spawn=True)
    
    # Create and send blueprints
    default, vision = create_blueprints()
    
    # Send blueprints
    rr.send_blueprint(default, make_default=True, make_active=True)
    rr.send_blueprint(vision, make_default=False, make_active=False)
    
    print("Rerun viewer started and ready for GRPC stream")
    print("Blueprints applied successfully")
    
    # Keep the script running to maintain the connection
    try:
        print("Viewer is running. Press Ctrl+C to exit.")
        while True:
            pass
    except KeyboardInterrupt:
        print("\nViewer stopped.")

def load_log_with_blueprints(log_file_path):
    """Load a rerun log file and apply custom blueprints"""
    
    if not os.path.exists(log_file_path):
        print(f"Error: Log file '{log_file_path}' does not exist")
        return
    
    APP_ID = "fw_salvador_file"
    
    # Initialize rerun
    rr.init(APP_ID, spawn=True)
    rr.log_file_from_path(log_file_path)
    
    # Create and send blueprints
    default, vision = create_blueprints()
    
    # Send blueprints
    rr.send_blueprint(default, make_default=True, make_active=True)
    rr.send_blueprint(vision, make_default=False, make_active=False)
    
    print(f"Loaded log file: {log_file_path}")
    print("Blueprints applied successfully")

if __name__ == "__main__":
    if len(sys.argv) == 1:
        # No arguments - start GRPC viewer
        start_grpc_viewer()
    elif len(sys.argv) == 2:
        # One argument - load log file
        log_file_path = sys.argv[1]
        load_log_with_blueprints(log_file_path)
    else:
        print("Usage: python rerun_local.py [path_to_log_file]")
        print("  No arguments: Start rerun viewer for GRPC stream")
        print("  With log file: python rerun_local.py /path/to/your/logfile.rrd")
        sys.exit(1)
