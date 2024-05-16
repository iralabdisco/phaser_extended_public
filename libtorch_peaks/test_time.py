import subprocess
import time

def run_cpp_executable(batch_size, tensor_size):
    start_time = time.time()
    subprocess.run(["/root/build/phaser_peaks", str(batch_size), str(tensor_size)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
    end_time = time.time()
    return end_time - start_time

def main():
    num_runs = 100
    batch_size = 1
    tensor_size = 240
    total_time = 0

    for i in range(num_runs):
        execution_time = run_cpp_executable(batch_size, tensor_size)
        total_time += execution_time

    avg_time = total_time / num_runs
    print(f"\nAverage Execution Time: {avg_time:.4f} seconds")
if __name__ == "__main__":
    main()
