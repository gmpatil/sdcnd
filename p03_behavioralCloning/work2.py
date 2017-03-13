from model import evaluate_model


training_data_folder = '/home/girish/dev/sdcnd/CarND-Behavioral-Cloning-P3/train4/'
csv_file_name = 'driving_log.csv'
model_file_name = "model.h5"

csv_file_path = training_data_folder + csv_file_name

test_loss = evaluate_model(model_file_name, training_data_folder, csv_file_path)

print (test_loss)