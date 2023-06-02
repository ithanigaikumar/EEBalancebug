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
chunks = [
    '97c28003',
    '16e2a010',
    '1662c017',
    '1622e01b',
    '15c30021',
    '15832025',
    '1523402a',
    '14a36030',
    '14438035',
    '1423a037',
    '13a3c03c',
    '1343e041',
    '13240043',
    '12e42045',
    '12c44047',
    '12a46049',
    '1284804b',
    '1264a04d',
    '1244c04f',
    '1224e051',
    '12050053',
    '11e52054',
    '11c54055',
    '11a56058',
    '11858059',
    '1165a05a',
    '1145c05c',
    '1125e05d',
    '1126005e',
    '1106205f',
    '10e64061',
    '10e66061',
    '10c68063',
    '10a6a065',
    '1086c066',
    '1086e066',
    '10870067',
    '10672068',
    '10674068',
    '10476069',
    '1047806a',
    '1027a06b',
    '1027c06b',
    '1027e06b',
    '1028006b',
    '1028206b',
    '1028406b',
    '1008606c',
    '1008806c',
    '1008a06d',
    '0fe8c06e',
    '0fe8e06e',
    '0fe9006f',
    '0fe9206f',
    '0fe9406f',
    '0fe9606f',
    '0fe9806f',
    '0fe9a06f',
    '0fc9c070',
    '0fe9e06f',
    '0fea006f',
    '0fea206e',
    '0fca406f',
    '0fea606e',
    '0fea806e',
    '0feaa06e',
    '0feac06e',
    '0feae06d',
    '0feb006d',
    '0feb206d',
    '0feb406d',
    '100b606c',
    '102b806a',
    '102ba06a',
    '102bc069',
    '102be069',
    '102c0069',
    '104c2067',
    '104c4066',
    '106c6065',
    '106c8064',
    '106ca064',
    '108cc062',
    '108ce061',
    '10ad005f',
    '10cd205d',
    '10cd405d',
    '10ed605b',
    '110d8059',
    '112da057',
    '112dc056',
    '114de054',
    '114e0052',
    '116e2051',
    '11ae404e',
    '11ae604d',
    '11ce804b',
    '11cea049',
    '11eec046',
    '122ee043',
    '126f003f',
    '128f203d',
    '12af403b',
    '12cf6038',
    '132f8032',
    '134fa030',
    '13cfc028',
    '142fe023',
    '1490001a',
    '15302011',
    '15d04001',
    '16304004',
]
process_chunks(chunks)