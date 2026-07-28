#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>

#include "loop_backend.h"

namespace {

void printUsage(const char* program) {
  std::cout
      << "Usage:\n  " << program
      << " --keyframe_dir <keyframes_robotdog_dir> [--config <yaml>]"
         " [--output_dir <dir>]\n\n"
         "Example:\n  "
      << program
      << " --keyframe_dir Log/pcd/keyframes_robotdog"
         " --config config/loop_backend_robotdog.yaml"
         " --output_dir Log/loop_optimized\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string keyframe_dir;
  std::string config_path;
  std::string output_dir = "Log/loop_optimized";

  for (int i = 1; i < argc; ++i) {
    const std::string argument(argv[i]);
    if (argument == "--help" || argument == "-h") {
      printUsage(argv[0]);
      return EXIT_SUCCESS;
    }
    if (argument == "--keyframe_dir" || argument == "--config" ||
        argument == "--output_dir") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << argument << std::endl;
        printUsage(argv[0]);
        return EXIT_FAILURE;
      }
      const std::string value(argv[++i]);
      if (argument == "--keyframe_dir") {
        keyframe_dir = value;
      } else if (argument == "--config") {
        config_path = value;
      } else {
        output_dir = value;
      }
      continue;
    }

    std::cerr << "Unknown argument: " << argument << std::endl;
    printUsage(argv[0]);
    return EXIT_FAILURE;
  }

  if (keyframe_dir.empty()) {
    std::cerr << "--keyframe_dir is required." << std::endl;
    printUsage(argv[0]);
    return EXIT_FAILURE;
  }

  try {
    LoopBackendConfig config;
    if (!config_path.empty() && !LoopBackend::loadConfigFile(config_path, &config)) {
      std::cerr << "Unable to load configuration: " << config_path << std::endl;
      return EXIT_FAILURE;
    }

    LoopBackend backend(config);
    if (!backend.loadKeyframes(keyframe_dir)) {
      std::cerr << "Unable to load keyframes from: " << keyframe_dir << std::endl;
      return EXIT_FAILURE;
    }
    if (!backend.optimize()) {
      std::cerr << "Pose-graph optimization failed." << std::endl;
      return EXIT_FAILURE;
    }
    if (!backend.exportResults(output_dir)) {
      std::cerr << "Unable to export optimization results to: " << output_dir
                << std::endl;
      return EXIT_FAILURE;
    }

    std::cout << "[loop_optimizer] Done. " << backend.loopEdges().size()
              << " loop edge(s) accepted." << std::endl;
    return EXIT_SUCCESS;
  } catch (const std::exception& exception) {
    std::cerr << "[loop_optimizer] Fatal error: " << exception.what() << std::endl;
    return EXIT_FAILURE;
  }
}
