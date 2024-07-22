/*
Author: Jhanvi Shah
==============================================================================*/

constexpr int numCols = 32; // 32
constexpr int numRows = 32; // 32 
constexpr int numChannels = 3; // 3

constexpr int maxImageSize = numCols * numRows * numChannels;

constexpr int categoryCount = 2;
constexpr int birdIndex = 1;
constexpr int notBirdIndex = 0;
extern const char* kCategoryLabels[categoryCount];

extern const unsigned int birdModelDataLength;
extern const unsigned char birdModelData[];