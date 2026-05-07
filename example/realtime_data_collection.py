import time
from sdk_wrapper import wrapper
from sdk_types import PY_7I_RESOLUTION

def main():
    print("Initializing SDK...")
    sdk = wrapper()
    
    # Path to config must point to the original bin directory or a copied config
    config_path = b'E:/7invensun/aSeeGlassesPlusUserSDK/bin/config'
    sdk.load_library(config_path)
    
    # Reading pwd from config.ini
    import configparser
    cf = configparser.ConfigParser()
    cf.read('E:/7invensun/aSeeGlassesPlusUserSDK/bin/config/config.ini')
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
