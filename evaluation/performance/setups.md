# Setups used for thesis:

## GPU
 * gpuConv.cpp -> bool gpuConvAdded = addGpuConv();
 * gpuFFT -> bool gpuFFTAdded = addGpuFFT();
 * DataProviders:
```
list.push_back(std::make_shared<FileDataProvider>("../images/MDB/MiddEval3-Q", "MDB-Q"));
list.push_back(std::make_shared<FileDataProvider>("../images/MDB/MiddEval3-H", "MDB-H"));
list.push_back(std::make_shared<FileDataProvider>("../images/MDB/MiddEval3-F", "MDB-F"));
```
 * command eval_performance -r 50 -nt -ns -np -hp
