import time
import configparser
import os

from example_paths import sdk_config_dir
from sdk_wrapper import wrapper

def main():
    print("Initializing SDK...")
    sdk = wrapper()

    config_path = os.fspath(sdk_config_dir())
    sdk.load_library(config_path)

    cf = configparser.ConfigParser()
    cf.read(os.path.join(config_path, "config.ini"))
    pwd = cf.get('softdog', 'pwd', fallback="").encode('utf-8')

    print(f"Connecting to softdog with pwd {pwd}...")
    ret = sdk.connect_softdog(pwd)
    if ret != 0:
        print("Failed to connect softdog")
        return

    print("Starting SDK...")
    # Environment: indoor(301), outdoor(302), darkness(303)
    # Resolution: 1280*720(202)
    ret = sdk.start(301, 202, 1280, 720)
    if ret != 0:
        print("Failed to start SDK")
        return

    print("Tracking gaze. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    sdk.stop()

if __name__ == "__main__":
    main()
