import os
import sqlite3
import json
import yaml
import sys

if __name__ == "__main__":
    file = "photon.sqlite"
    con = sqlite3.connect(file)
    con.autocommit = True
    cur = con.cursor()
    res = cur.execute("SELECT config_json FROM cameras WHERE unique_name = 'Arducam_OV2311_USB_Camera'")
    config = res.fetchone()
    data = json.loads(config[0])
    print(config[0][:200])
    calibrations = data.get("calibrations", None)
    new_calibration = {}
    if calibrations is not None:
        new_calibration = calibrations[0]
        with open("calib_3_front-right.yml", "r") as f:
            calibdb =  yaml.load(f, yaml.SafeLoader)
        # print(calibdb)
        new_calibration["resolution"]["width"] = calibdb.get("image_width")
        new_calibration["resolution"]["height"] = calibdb.get("image_height")
        new_calibration["cameraIntrinsics"] = calibdb.get("camera_matrix")
        new_calibration["cameraIntrinsics"]["type"] = 6
        new_calibration["distCoeffs"] = calibdb.get("distortion_coefficients")
        new_calibration["distCoeffs"]["type"] = 6
        new_calibration["observations"] = []
    data["calibrations"] = [new_calibration]
    string = json.dumps(data)
    print(string[:200])
    res = cur.execute(f"UPDATE cameras SET config_json='{string}' WHERE unique_name = 'Arducam_OV2311_USB_Camera'")
    # res = cur.execute("SELECT config_json FROM cameras WHERE unique_name = 'Arducam_OV2311_USB_Camera'")
    # a = res.fetchone()
    # print(a[0][:200])

    # print(config)




