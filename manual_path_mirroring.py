import json


with open("src/main/deploy/pathplanner/bot-2-gp-blue.path", "r") as f:
    path = json.load(f)
offset = 16.53
for waypoint in path["waypoints"]:
    if waypoint["anchorPoint"] is not None:
        waypoint["anchorPoint"]["x"] = offset - waypoint["anchorPoint"]["x"]

    if waypoint["prevControl"] is not None:
        waypoint["prevControl"]["x"] = offset - waypoint["prevControl"]["x"]

    if waypoint["nextControl"] is not None:
        waypoint["nextControl"]["x"] = offset - waypoint["nextControl"]["x"]

with open("src/main/deploy/pathplanner/bot-2-gp-red.path", "w") as f:
    json.dump(path,f)