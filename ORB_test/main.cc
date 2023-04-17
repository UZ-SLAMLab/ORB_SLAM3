#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

#include "ORBAccel.h"

using namespace std;
using namespace cv;

int main() {
    Buffer in_buffer(1920*1080);
    Buffer out_buffer(MAX_OUTPUT_LENGTH*OUTPUT_BYTES);
    MMIO kernel(ORB_BASE);
    MMIO dma_mmio(DMA_BASE);
    DMA dma(&dma_mmio);
    int threshold = 40;

    for (int i = 0; i < 8; i++) {
        Mat image;
        // if (i == 0) {
        //     image = imread("tum_480p.png", IMREAD_GRAYSCALE);
        // } else {
        //     image = imread("000000.png", IMREAD_GRAYSCALE);
        // }
        image = imread("000000.png", IMREAD_GRAYSCALE);
        int length = image.rows * image.cols;
        
        in_buffer.copy(image.data, length*sizeof(uint8_t));

        dma.sendchannel.transfer(&in_buffer);
        dma.recvchannel.transfer(&out_buffer);

        kernel.write(ORB_HEIGHT, image.rows);
        kernel.write(ORB_WIDTH, image.cols);
        kernel.write(ORB_THRESHOLD, threshold);
        kernel.write(ORB_START, 0x1);

        dma.sendchannel.wait();
        dma.recvchannel.wait();

        int n_features = kernel.read(ORB_RETURN);

        cout << "number of features: " << n_features << endl;

        for (int i = 0; i < n_features; i++) {
            unsigned x, y;
            unsigned x_high = out_buffer.ptr[i*OUTPUT_BYTES + 35];
            unsigned x_low = out_buffer.ptr[i*OUTPUT_BYTES + 34];
            unsigned y_high = out_buffer.ptr[i*OUTPUT_BYTES + 33];
            unsigned y_low = out_buffer.ptr[i*OUTPUT_BYTES + 32];
            x = ((x_high << 8 & 0xFF00) | (x_low & 0x00FF));
            y = ((y_high << 8 & 0xFF00) | (y_low & 0x00FF));
            // printf("(%03d, %03d) ", x, y);
            // for (int j = 0; j < 32; j++) {
            //     unsigned tmp = out_buffer.ptr[i*OUTPUT_BYTES + 31 - j];
            //     printf("%02X", tmp);
            // }
            // printf("\n");
        }
    }

    in_buffer.free();
    out_buffer.free();
    kernel.close();
    dma_mmio.close();


    return 0;
}