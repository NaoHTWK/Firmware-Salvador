import rerun as rr, numpy as np, os
import rerun.blueprint as rrb


run_id = "fw_salvador_online" #os.getenv("RUN_ID")  
APP_ID = "fw_salvador_online"
SERVER_URL = "rerun+http://127.0.0.1:9876/proxy";

rr.init(APP_ID, recording_id=run_id)
rr.connect_grpc(SERVER_URL)  

#sending blueprint

#default blueprint
contents= ['+ /image/**']
topleft = rrb.Spatial2DView(name="Vision", contents=contents)
contents= ['+ /**']
topright = rrb.TextLogView(name="Text", contents=contents)

contents= ['+ /soccerfield/**', '- /soccerfield/team_com/**', '- /soccerfield/localization/hypotheses/**', "- /soccerfield/point_features/**", ]
bottomleft = rrb.Spatial2DView(name="Soccer Field", contents=contents,  background=[34, 139, 36])

contents=['+ /relative/**', '- /relative/no_obstacles']
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


vision = rrb.Blueprint(
    rrb.Horizontal(
        rrb.Spatial3DView(origin="vision", name="vision"),
        rrb.Spatial2DView(origin="vision", name="vision2")
    ),
    collapse_panels=True,

)

rr.send_blueprint(default, make_default=True, make_active=True)
rr.send_blueprint(vision, make_default=False, make_active=False)

print("python script has been executed")
rr.log("py/text/logs", rr.TextLog("python script has been executed", level=rr.TextLogLevel.TRACE))
