from pzem017 import PZEM017
import time

pz = PZEM017(port="COM3", addr=0x01)

while True:
    try:
        data = pz.read_values()
        print(
            f"V={data['voltage']:.2f}V  "
            f"I={data['current']:.2f}A  "
            f"P={data['power']:.1f}W  "
            f"E={data['energy']:.3f}kWh"
        )
    except Exception as e:
        print("Error:", e)

    time.sleep(1)
