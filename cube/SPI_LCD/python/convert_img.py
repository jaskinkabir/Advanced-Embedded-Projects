from PIL import Image
import sys

def convert_png_to_c_header(png_file, header_file):
    try:
        # Open the image file
        img = Image.open(png_file)
        img = img.convert("RGB")  # Ensure the image is in RGB mode

        # Get image dimensions
        width, height = img.size

        # Prepare the header file
        with open(header_file, "w") as f:
            f.write("// Generated C header file\n")
            f.write("// Image dimensions: {}x{}\n".format(width, height))
            f.write("#ifndef IMAGE_DATA_H\n#define IMAGE_DATA_H\n\n")
            f.write("#define IMAGE_WIDTH {}\n".format(width))
            f.write("#define IMAGE_HEIGHT {}\n\n".format(height))
            f.write("const uint16_t image_data[IMAGE_HEIGHT][IMAGE_WIDTH] = {\n")

            # Convert each pixel to 16-bit RGB565 and write to the header file
            for y in range(height):
                f.write("    { ")
                for x in range(width):
                    r, g, b = img.getpixel((x, y))

                    # Convert to RGB565
                    rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)

                    # Write the value as a hex literal
                    f.write("0x{:04X}".format(rgb565))

                    if x < width - 1:
                        f.write(", ")
                f.write(" },\n")

            f.write("};\n\n")
            f.write("#endif // IMAGE_DATA_H\n")

        print("Header file '{}' generated successfully.".format(header_file))

    except Exception as e:
        print("Error: {}".format(e))

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python convert_img.py <input_png_file> <output_header_file>")
    else:
        png_file = sys.argv[1]
        header_file = sys.argv[2]
        convert_png_to_c_header(png_file, header_file)
