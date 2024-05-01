
#Check if Slic3r is installed

if ! command -v slic3r &> /dev/null; then
	echo "Slic3r is not installed. Please install it and try again"
	exit 1
fi

#Check if an input file is provided 
if [ -z "$1" ]; then
	echo "Usage: $0 <input)file.stl>"
	exit 1
fi

#Check if the input file exists 
if [ ! -f "$1" ]; then 
	echo "File '$1' not found."
	exit 1
fi

#Set output directory
output_dir="DroneGcodeOutput"
mkdir -p "$output_dir"

#Get the base filename
filename=$(basename -- "$1")
filename_no_ext="${filename%.*}"

#Set output file path
export output_file="$output_dir/$filename_no_ext.gcode"

#Slice the STL file to G-code using Slic3r
echo "Slicing $1 to $output_file..."
if slic3r --load config.ini --start-gcode start.gcode --end-gcode end.gcode  --before-layer-gcode beforelayer.gcode  --output "$output_file" "$1"; then
    echo "Slicing completed. Output saved to $output_file."
else
    echo "Error occurred while slicing."
    exit 1
fi
#Send Gcode
python3 printer.py "$output_file"
