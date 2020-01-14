import subprocess
import sys


detectors = ["SHITOMASI", "HARRIS", "BRISK", "ORB", "FAST", "AKAZE", "SIFT"]
descriptors = ["BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"]

print("Detector,Descriptor,Selector,Img,TTC-Lidar,TTC-Cam,TTC-diff", file=sys.stderr)

for det in detectors:
    for desc in descriptors:
        for sel in ["SEL_NN", "SEL_KNN"]:
            print(f"Executing with {det} {desc} MAT_BF {sel}")
            ps = subprocess.run(['../build/3D_object_tracking', det, desc, 'MAT_BF', sel],
                                capture_output=True, text=True)

            if ps.returncode != 0:
                print(f"{det} + {desc} + {sel} combo is invalid")
                continue

            out = ps.stderr
            print(out, file=sys.stderr)


