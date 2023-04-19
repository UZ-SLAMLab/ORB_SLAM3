#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include "ORBAccel.h"

using namespace std;
using namespace cv;

int main() {
    Buffer in_buffer(1920*1080);
    Buffer out_buffer(MAX_OUTPUT_LENGTH*OUTPUT_BYTES);
    MMIO kernel(ORB_BASE);
    // MMIO dma_mmio(DMA_BASE);
    DMA dma(DMA_BASE);
    int threshold = 40;

    for (int n = 0; n < 10; n++) {
        Mat image;
        image = imread("tum_480p.png", IMREAD_GRAYSCALE);
        // image = imread("000000.png", IMREAD_GRAYSCALE);
        int nAllfeatures = 0;
        int cols = image.cols;
        int rows = image.rows;
        int length = cols * rows;
        in_buffer.copy(image.data, length*sizeof(uint8_t));
        double scale = 1;

        for (int i = 0; i < 8; i++) {     
            if (i > 0) {
                scale = scale / 1.2;
            }  
            int new_cols = scale * cols;
            int new_rows = scale * rows;
            cout << "level " << i << " size: " << new_cols << " " << new_rows << endl;

            dma.sendchannel.transfer(&in_buffer);
            dma.recvchannel.transfer(&out_buffer);

            kernel.write(ORB_HEIGHT, rows);
            kernel.write(ORB_WIDTH, cols);
            kernel.write(ORB_HEIGHT_NEW, new_rows);
            kernel.write(ORB_WIDTH_NEW, new_cols);
            kernel.write(ORB_THRESHOLD, threshold);
            kernel.write(ORB_START, 0x1);

            dma.sendchannel.wait();
            dma.recvchannel.wait();

            int n_features = kernel.read(ORB_RETURN);
            nAllfeatures += n_features;

            cout << "number of features: " << n_features << endl;
            cout << "all features: " << nAllfeatures << endl;

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

        
    }
    in_buffer.free();
    out_buffer.free();
    kernel.close();
    dma.close();
    return 0;
}
