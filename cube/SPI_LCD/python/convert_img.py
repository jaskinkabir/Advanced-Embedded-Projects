from PIL import Image
import sys

def convert_png_to_c_header(png_file, header_file):
    try:
        # Open the image file
        img = Image.open(png_file)
        img = img.convert("RGB")  # Ensure the image is in RGB mode

        # Get image dimensions
        width, height = img.size

        # Extract the base name of the image file (without extension)
        image_name = png_file.split(".")[0]

        # Prepare the header file
        with open(header_file, "w") as f:
            f.write("// Generated C header file\n")
            f.write(f"// Image dimensions: {width}x{height}\n")
            f.write(f"#ifndef {image_name.upper()}_H\n#define {image_name.upper()}_H\n\n")
            f.write(f"#define IMAGE_WIDTH {width}\n")
            f.write(f"#define IMAGE_HEIGHT {height}\n\n")
            f.write(f"const uint16_t {image_name}[IMAGE_WIDTH * IMAGE_HEIGHT] = {{\n")

            # Convert each pixel to 16-bit RGB565 and write to the header file
            for y in range(height):
                for x in range(width):
                    r, g, b = img.getpixel((x, y))

                    # Convert to RGB565
                    rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)

                    # Write the value as a hex literal
                    f.write(f"0x{rgb565:04X}, ")
                f.write("\n")

            f.write("};\n\n")
            f.write(f"#endif // {image_name.upper()}_H\n")

        print(f"Header file '{header_file}' generated successfully.")

    except FileExistsError as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python convert_img.py <input_png_file> <output_header_file>")
    else:
        png_file = sys.argv[1]
        header_file = sys.argv[2]
        convert_png_to_c_header(png_file, header_file)
