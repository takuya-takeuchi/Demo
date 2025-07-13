#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>

#include "ort_genai.h"

static void print_usage(int /*argc*/, char** argv)
{
  std::cerr << "usage: " << argv[0] << " <model_path> <execution_provider>" << std::endl;
  std::cerr << "  model_path: [required] Path to the folder containing onnx models, genai_config.json, etc." << std::endl;
  std::cerr << "  execution_provider: [optional] Force use of a particular execution provider (e.g. \"cpu\")" << std::endl;
  std::cerr << "                      If not specified, EP / provider options specified in genai_config.json will be used." << std::endl;
}

bool parse_args(int argc, char** argv, std::string& model_path, std::string& ep)
{
  if (argc < 2) {
    print_usage(argc, argv);
    return false;
  }
  model_path = argv[1];
  if (argc > 2) {
    ep = argv[2];
  } else {
    ep = "follow_config";
  }
  return true;
}

bool fileExists(const char* path)
{
  return static_cast<bool>(std::ifstream(path));
}

std::string trim(const std::string& str)
{
  const size_t first = str.find_first_not_of(' ');
  if (std::string::npos == first)
    return str;
  const size_t last = str.find_last_not_of(' ');
  return str.substr(first, (last - first + 1));
}

void append_provider(OgaConfig& config, const std::string& provider)
{
  if (provider.compare("follow_config") == 0)
    return;

  config.ClearProviders();
  if (provider.compare("cpu") == 0)
    return;

  config.AppendProvider(provider.c_str());
  if (provider.compare("cuda") == 0)
  {
    config.SetProviderOption(provider.c_str(), "enable_cuda_graph", "0");
  }
}

void CXX_API(const char* model_path, const char* execution_provider)
{
  std::cout << "Creating config..." << std::endl;
  auto config = OgaConfig::Create(model_path);

  std::string provider(execution_provider);
  append_provider(*config, provider);

  std::cout << "Creating model..." << std::endl;
  auto model = OgaModel::Create(*config);

  std::cout << "Creating multimodal processor..." << std::endl;
  auto processor = OgaMultiModalProcessor::Create(*model);

  auto tokenizer_stream = OgaTokenizerStream::Create(*processor);

  while (true)
  {
    std::string image_paths_str;
    std::cout << "Image Path (comma separated; leave empty if no image):" << std::endl;
    std::getline(std::cin, image_paths_str);
    std::unique_ptr<OgaImages> images;
    std::vector<std::string> image_paths;
    for (size_t start = 0, end = 0; end < image_paths_str.size(); start = end + 1)
    {
      end = image_paths_str.find(',', start);
      image_paths.push_back(trim(image_paths_str.substr(start, end - start)));
    }

    if (image_paths.empty())
    {
      std::cout << "No image provided" << std::endl;
    }
    else
    {
      std::cout << "Loading images..." << std::endl;
      for (const auto& image_path : image_paths)
      {
        if (!fileExists(image_path.c_str())) {
          throw std::runtime_error(std::string("Image file not found: ") + image_path);
        }
      }

      std::vector<const char*> image_paths_c;
      for (const auto& image_path : image_paths) image_paths_c.push_back(image_path.c_str());
      images = OgaImages::Load(image_paths_c);
    }

    std::string text;
    std::cout << "Prompt: " << std::endl;
    std::getline(std::cin, text);
    std::string prompt = "<|user|>\n";
    if (images)
    {
      for (size_t i = 0; i < image_paths.size(); ++i)
        prompt += "<|image_" + std::to_string(i + 1) + "|>\n";
    }
    prompt += text + "<|end|>\n<|assistant|>\n";

    std::cout << "Processing images and prompt..." << std::endl;
    auto input_tensors = processor->ProcessImages(prompt.c_str(), images.get());

    std::cout << "Generating response..." << std::endl;
    auto params = OgaGeneratorParams::Create(*model);
    params->SetSearchOption("max_length", 7680);
    params->SetInputs(*input_tensors);

    auto generator = OgaGenerator::Create(*model, *params);

    while (!generator->IsDone())
    {
      generator->GenerateNextToken();

      const auto num_tokens = generator->GetSequenceCount(0);
      const auto new_token = generator->GetSequenceData(0)[num_tokens - 1];
      std::cout << tokenizer_stream->Decode(new_token) << std::flush;
    }

    for (int i = 0; i < 3; ++i)
      std::cout << std::endl;
  }
}

int main(int argc, char** argv) {
  std::string model_path, ep;
  if (!parse_args(argc, argv, model_path, ep))
  {
    return -1;
  }

  std::cout << "--------------------" << std::endl;
  std::cout << "Hello, Phi-3-Vision!" << std::endl;
  std::cout << "--------------------" << std::endl;

  std::cout << "C++ API" << std::endl;
  CXX_API(model_path.c_str(), ep.c_str());

  return 0;
}