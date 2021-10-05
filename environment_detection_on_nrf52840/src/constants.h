/*
Contains the environments to classify between, mean and std values for normalization and the pretrained neural network model.
*/


#ifndef TENSORFLOW_LITE_MICRO_EXAMPLES_HELLO_WORLD_CONSTANTS_H_
#define TENSORFLOW_LITE_MICRO_EXAMPLES_HELLO_WORLD_CONSTANTS_H_

//environments that can be detected by the neural network
extern const char available_env[][50];
extern const int available_env_len;

//mean and std values for all 46 features to normalize input data
extern const float mean_list[46];
extern const float std_list[46];

//pre trained neural network
extern const unsigned char g_modelurd[];
extern const int g_model_len;

#endif 
