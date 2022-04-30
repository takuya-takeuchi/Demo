from concurrent import futures
import time

import grpc

from PIL import Image
import PIL.ImageOps
import imageProc_pb2
import imageProc_pb2_grpc 

_ONE_DAY_IN_SECONDS = 60 * 60 * 24

class ImageProc(imageProc_pb2_grpc.ImageProcServicer):
    def Enhancement(self, request, context):
        reply = imageProc_pb2.EnhancementReply()
        reply.width = request.width
        reply.height = request.height
        reply.channel = request.channel

        imageSize = request.width, request.height
        # 1 (1-bit pixels, black and white, stored with one pixel per byte)
        # L (8-bit pixels, black and white)
        # P (8-bit pixels, mapped to any other mode using a colour palette)
        # RGB (3x8-bit pixels, true colour)
        # RGBA (4x8-bit pixels, true colour with transparency mask)
        # CMYK (4x8-bit pixels, colour separation)
        # YCbCr (3x8-bit pixels, colour video format)
        # I (32-bit signed integer pixels)
        # F (32-bit floating point pixels)
        channel = request.channel
        if channel == 4:
            print('channel is 4')
            # PIL.ImageOps.invert does NOT support RGBA
            tmp = Image.frombytes('RGBA',  imageSize, request.image)
            r, g, b, a = tmp.split()
            rgb = Image.merge("RGB", (b, g, r))
            inverted = PIL.ImageOps.invert(rgb)
            r, g, b = inverted.split()
            reply.image = Image.merge("RGBA", (r, g, b, a)).tobytes()
        elif channel == 3:
            print('channel is 3')
            reply.image = PIL.ImageOps.invert(Image.frombytes('RGB', imageSize, request.image)).tobytes()
        elif channel == 1:
            print('channel is 1')
            reply.image = PIL.ImageOps.invert(Image.frombytes('1', imageSize, request.image)).tobytes()
        else:
            print('channel is unknown')
            reply.image = PIL.ImageOps.invert(Image.frombytes('L', imageSize, request.image)).tobytes()

        reply.result = 0
        return reply


def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    imageProc_pb2_grpc.add_ImageProcServicer_to_server(ImageProc(), server)
    server.add_insecure_port('[::]:50051')
    server.start()
    try:
        while True:
            time.sleep(_ONE_DAY_IN_SECONDS)
    except KeyboardInterrupt:
        server.stop(0)

if __name__ == '__main__':
  serve()
