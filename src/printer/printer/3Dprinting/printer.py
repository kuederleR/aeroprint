import subprocess
import PrusaLinkPy
import os
import sys

def main(output_file):
    # Constructing the full path to the G-code file
    path = os.path.dirname(os.path.abspath(__file__))  
    gcode_path = os.path.join(path, output_file)

    prusaMK4 = PrusaLinkPy.PrusaLinkPy("192.168.8.181", "9BQHmQ5SRVc5RRE")

    getPrint = prusaMK4.get_printer()
    print(getPrint.json()["telemetry"])

    # Path to Gcode
    printer_path = os.path.join("/usb/aeroprint/" + output_file)

    if prusaMK4.exists_gcode(printer_path):
        prusaMK4.delete(printer_path)
    prusaMK4.put_gcode(gcode_path, printer_path)

    prusaMK4.post_print_gcode(printer_path)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <output_file>")
        sys.exit(1)
    output_file = sys.argv[1]
    main(output_file)
