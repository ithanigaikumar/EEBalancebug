from PIL import Image

def convert_to_3bit_color(image_path):
    # Open the image
    image = Image.open(image_path)

    # Convert the image to RGB mode
    image = image.convert("RGB")

    # Create a new palette with the desired colors
    palette = Image.new("P", (1, 1), 0)
    palette.putpalette([
        0, 0, 0,         # Black
        255, 255, 255,   # White
        255, 0, 0,       # Red
        0, 255, 0,       # Green
        0, 0, 255,       # Blue
        0, 255, 255,     # Cyan
        255, 0, 255,     # Magenta
        255, 255, 0      # Yellow
    ])

    # Convert the image to 3-bit color using the custom palette
    image = image.quantize(colors=8, palette=palette)

    # Save the converted image
    converted_image_path = image_path.replace(".png", "_3bit.png")
    image.save(converted_image_path)

    print(f"Image converted and saved as: {converted_image_path}")

# Provide the path to your image
image_path = "bec.png"

# Call the function to convert the image
convert_to_3bit_color(image_path)
