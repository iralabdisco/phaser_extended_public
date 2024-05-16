#include <torch/torch.h>
#include <iostream>

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " batch_size tensor_size" << std::endl;
        return 1;
    }

    int batch_size = std::stoi(argv[1]);
    int tensor_size = std::stoi(argv[2]);

    torch::TensorOptions options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA);
    torch::Tensor tensor = torch::ones({batch_size, tensor_size, tensor_size, tensor_size});

    // Specify four locations where the value is 2
    tensor[0][0][0][0] = 2;          // Example location 1
    tensor[0][2][2][2] = 2;    // Example location 2

    //std::cout << "original_tensor: " << tensor;

    // Define a 3D max pooling layer with kernel size 3 and stride 1
    torch::nn::MaxPool3d maxpool(torch::nn::MaxPool3dOptions(7).stride(1).padding(3));

    // Apply max pooling to the tensor
    torch::Tensor pooled_tensor = maxpool(tensor);
    //std::cout << "pooled_tensor" << pooled_tensor;

    // Find the locations where the pooled tensor is equal to the original tensor
    torch::Tensor maxima_mask = (torch::isclose(tensor,pooled_tensor)).to(torch::kByte);
    //std::cout << "maxima_max" << maxima_mask;

    // Get the indices of non-zero elements in the maxima mask
    torch::Tensor indices = torch::nonzero(maxima_mask);

     // Output the indices of local maxima
    // for (int i = 0; i < indices.size(0); ++i) {
    //     std::cout << "Local maximum at: (" << indices[i][0].item<int>() << ", "
    //                                        << indices[i][1].item<int>() << ", "
    //                                        << indices[i][2].item<int>() << ")\n";
    // }

    return 0;
}
