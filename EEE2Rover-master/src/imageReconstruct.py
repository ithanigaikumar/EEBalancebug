from PIL import Image

def process_chunks(chunks, width=640, height=254):
    # Initialize an all black image
    frame = Image.new('1', (width, height))

    # Process each chunk
    for chunk in chunks:
        # Convert hex to binary
        binary = format(int(chunk, 16), '032b')

        # Unpack the chunk
        new_frame = int(binary[0], 2)
        x_start = int(binary[1:11], 2)
        y_start = int(binary[11:19], 2)
        chunk_length = int(binary[19:], 2)

        # If this chunk starts a new frame, reinitialize the frame
        if new_frame:
            frame = Image.new('1', (width, height))

        # Draw the white pixels
        for x in range(x_start, x_start + chunk_length):
            # Ensure we're not drawing outside the frame
            if x < width and y_start < height:
                frame.putpixel((x, y_start), 1)

    # Show the final frame
    frame.show()

# Sample chunk data of one frame in hex

process_chunks(chunks)