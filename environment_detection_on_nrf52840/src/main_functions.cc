/*
Using a predefined neural network we predict the environment to a given data sample.
*/


#include "main_functions.h"

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "constants.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include <sys/printk.h>
#include <math.h>

#include <zephyr.h>

#define DATA_LINE_LENGTH 46
#define DATA_ROWS 5

static float prepared_data[DATA_LINE_LENGTH*DATA_ROWS];


namespace
{
tflite::ErrorReporter *error_reporter = nullptr;
const tflite::Model *model = nullptr;
tflite::MicroInterpreter *interpreter = nullptr;
TfLiteTensor *input = nullptr;
TfLiteTensor *output = nullptr;

// Create an area of memory to use for input, output, and intermediate arrays.
const int kModelArenaSize = 6064;
// Extra headroom for model + alignment + future interpreter changes.
const int kExtraArenaSize = 560 + 16 + 160;
const int kTensorArenaSize = kModelArenaSize + kExtraArenaSize;
static uint8_t tensor_arena[kTensorArenaSize];
} 

/*
normalize data sample; must be done since neural network is trained with normalized data
*/
void prepare_data(int raw_data[], int length, int rows, float *prepared)
{
	for (int i = 0; i < length; i++) {
		if (std_list[i] != 0) {
			for (int j = 0; j<rows; j++){
				
				prepared[length*j + i] = (raw_data[length*j + i] - mean_list[i]) / std_list[i];
				
				
			}
			
		}
	}
}

/*
initialize neural network
*/
void setup()
{

	static tflite::MicroErrorReporter micro_error_reporter;
	error_reporter = &micro_error_reporter;

	model = tflite::GetModel(g_modelurd);

	if (model->version() != TFLITE_SCHEMA_VERSION) {
		TF_LITE_REPORT_ERROR(error_reporter,
				     "Model provided is schema version %d not equal "
				     "to supported version %d.",
				     model->version(), TFLITE_SCHEMA_VERSION);
		printk("model not supported\n");
		return;
	}

	static tflite::AllOpsResolver resolver;

	// Build an interpreter to run the model with.
	static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena,
							   kTensorArenaSize, error_reporter);
	interpreter = &static_interpreter;

	// Allocate memory from the tensor_arena for the model's tensors.
	TfLiteStatus allocate_status = interpreter->AllocateTensors();

	if (allocate_status != kTfLiteOk) {
		TF_LITE_REPORT_ERROR(error_reporter, "AllocateTensors() failed");
		printk("tensor allocation failed\n");

		return;
	}

	// Obtain pointers to the model's input and output tensors.
	input = interpreter->input(0);
	output = interpreter->output(0);

	//print expected and actual input size and used memory 
	int expected = input->dims->data[1];
	printk("sample input: %d, expected input: %d, used tensor bytes: %d\n", DATA_LINE_LENGTH*DATA_ROWS, expected, interpreter->arena_used_bytes());
}

/*
predict data sample with pretrained neural network
*/
void loop(int data_sample[DATA_LINE_LENGTH*DATA_ROWS], struct classification *ptr)
{
	prepare_data(data_sample, DATA_LINE_LENGTH, DATA_ROWS, prepared_data);

	for (int i = 0; i < DATA_LINE_LENGTH*DATA_ROWS; i++) {
		input->data.f[i] = prepared_data[i];
	}

	//execute network
	interpreter->Invoke();
	float max_value = 0;
	int env_index_pred = -1;

	//find environment with highest probability
	for (int i = 0; i<available_env_len; i++){
		float pred = output->data.f[i];
		if( pred> max_value){
			max_value = pred;
			env_index_pred = i;
		}
	}
	

	ptr->index = env_index_pred;
	ptr->probability = max_value;
}
