import pandas as pd
import numpy as np
from sklearn.impute import SimpleImputer
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt
import csv
import sys
from PyQt5.QtWidgets import QApplication, QDialog, QVBoxLayout, QPushButton


class Plotter:
    def __init__(self):
        # List to store the plots and regression lines
        self.plots = []
        self.regression_lines = []

        # Create a figure and axis for the cumulative plot
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.data_paths = []

    def create_plots(self, data_path, pickup_time, travel_time):

        # Impute missing values with the mean
        imputer = SimpleImputer(strategy='mean')
        travel_time = imputer.fit_transform(travel_time.values.reshape(-1, 1))

        # Fit Linear Regression model
        regression_model = LinearRegression()
        regression_model.fit(pickup_time.values.astype('int64').reshape(-1, 1), travel_time)
       # Predict travel time
        predicted_travel_time = regression_model.predict(pickup_time.values.astype('int64').reshape(-1, 1))
                 # Add the regression line to the cumulative plot with corresponding dataset time
        self.ax.plot(pickup_time, predicted_travel_time, linewidth=1,
                     label=f'Dataset {self.data_paths.index(data_path) + 1} ')


        # Plot the data and regression line for each dataset
        plt.figure(figsize=(10, 6))
        plt.scatter(pickup_time, travel_time, marker='o', s=5, color='black', label='Actual travel time')
        plt.plot(pickup_time, predicted_travel_time, color='red', linewidth=1, label='Regression line')
        slope, slope = regression_model.coef_[0][0], regression_model.intercept_[0]
        plt.text(slope, slope, r'slope='+str(slope),color='blue', fontsize=8)
        plt.xlabel('Pickup Time')
        plt.ylabel('Travel Time (seconds)')
        plt.title('Linear Regression: Travel Time - Dataset ' + str(self.data_paths.index(data_path) + 1))
        plt.legend()
        self.plots.append(plt)

        # Store regression line parameters
        self.regression_lines.append((regression_model.coef_[0][0], regression_model.intercept_[0]))

    def show_plots(self):
          # Customize cumulative plot
        self.ax.set_xlabel('Pickup Time')
        self.ax.set_ylabel('Travel Time (seconds)')
        self.ax.set_title('Cumulative Linear Regression: Travel Time')
        self.ax.legend()

        # Display all plots together
        for plot in self.plots:
            plot.show()


        # Print regression line parameters
        for i, line in enumerate(self.regression_lines, start=1):
            slope, intercept = line
            print(f'Regression line equation for Dataset {i}: y = {slope:.2f}x + {intercept:.2f}')

    def prepare_dict_from_file(self, csv_file):
        walk_dict = {}
        with open(csv_file, newline='') as csvfile:
            reader = csv.DictReader(csvfile)

            for row in reader:
                walk_dict[row["request_id"]]= [row["walking_distance_from_stop"], row["walking_distance_to_stop"]]

            return walk_dict

        return None


class SimuPlot(Plotter):
    def __init__(self):
        super().__init__()

    def case1(self, data_paths):
        self.data_paths = data_paths
        for data_path in data_paths:
            data = pd.read_csv(data_path)
            pickup_time = data["pickup_time"]
            dropoff_time = data["dropoff_time"]
            travel_time = (dropoff_time - pickup_time)
            super().create_plots(data_path, pickup_time, travel_time)
        self.show_plots()

    def case2(self, data_paths, walking_dist_file):
        self.data_paths = data_paths
        walking_dict = self.prepare_dict_from_file(walking_dist_file)
        for data_path in data_paths:
            data = pd.read_csv(data_path)

            for key, value in walking_dict.items():
                if int(key) in data["request_id"].to_dict().values():
                    data["dropoff_time"] = data["dropoff_time"] + int(value[0])+ int(value[1])

            pickup_time = data["pickup_time"]
            dropoff_time = data["dropoff_time"]
            travel_time = (dropoff_time - pickup_time)

            super().create_plots(data_path, pickup_time, travel_time)
        self.show_plots()



def case1_fun():
    simu_plot = SimuPlot()
    # Load the data
    data_paths = [
        "//Users//rakeshsadhu//dev//mygit//work//languages//python//akshay_proj//data_set_case1//MiDN2NDemand_15//SUMO_FLEETPY_SIM//1_user-stats.csv",
        "//Users//rakeshsadhu//dev//mygit//work//languages//python//akshay_proj//data_set_case1//MiDN2NDemand_20//SUMO_FLEETPY_SIM//1_user-stats.csv",
        "//Users//rakeshsadhu//dev//mygit//work//languages//python//akshay_proj//data_set_case1//MiDN2NDemand_25//SUMO_FLEETPY_SIM//1_user-stats.csv"
    ]
    simu_plot.case1(data_paths)


def case2_fun():
    simu_plot = SimuPlot()
    # Load the data
    data_paths = [
        "//Users//rakeshsadhu//dev//mygit//work//languages//python//akshay_proj//data_set_case2//MiDS2SDemand_15//SUMO_FLEETPY_SIM//1_user-stats.csv",
        "//Users//rakeshsadhu//dev//mygit//work//languages//python//akshay_proj//data_set_case2//MiDS2SDemand_20//SUMO_FLEETPY_SIM//1_user-stats.csv",
        "//Users//rakeshsadhu//dev//mygit//work//languages//python//akshay_proj//data_set_case2//MiDS2SDemand_25//SUMO_FLEETPY_SIM//1_user-stats.csv",
    ]
    simu_plot.case2(data_paths, "//Users//rakeshsadhu//dev//mygit//work//languages//python//akshay_proj//data_set_case2//MiD_S2S_Walking_Distances.csv")



def case3_fun():
    simu_plot = SimuPlot()
    # Load the data
    data_paths = [
        "//Users//rakeshsadhu//dev//mygit//work//languages//python//akshay_proj//data_set_case1//MiDN2NDemand_15//SUMO_FLEETPY_SIM//1_user-stats.csv",
        "//Users//rakeshsadhu//dev//mygit//work//languages//python//akshay_proj//data_set_case1//MiDN2NDemand_20//SUMO_FLEETPY_SIM//1_user-stats.csv",
        "//Users//rakeshsadhu//dev//mygit//work//languages//python//akshay_proj//data_set_case1//MiDN2NDemand_25//SUMO_FLEETPY_SIM//1_user-stats.csv"
        # Add more data paths here for additional datasets
    ]
    simu_plot.case3(data_paths)








class MyDialog(QDialog):
    def __init__(self):
        super(MyDialog, self).__init__()

        self.init_ui()

    def init_ui(self):
        # Set up the layout
        layout = QVBoxLayout()

        # Add three buttons to the layout
        button1 = QPushButton('case 1', self)
        button2 = QPushButton('case 2', self)
        button3 = QPushButton('case 3', self)

        # Connect buttons to their respective cases
        button1.clicked.connect(self.case1)
        button2.clicked.connect(self.case2)
        button3.clicked.connect(self.case3)

        # Add buttons to the layout
        layout.addWidget(button1)
        layout.addWidget(button2)
        layout.addWidget(button3)

        # Set the layout for the dialog
        self.setLayout(layout)

        # Set the dialog properties
        self.setWindowTitle('Case Dialog')
        self.setGeometry(300, 300, 300, 150)

    def case1(self):
        print("case 1 executed!")
        case1_fun()

    def case2(self):
        print("case 2 executed!")
        case2_fun()

    def case3(self):
        print("case 3 executed!")
        case3_fun()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = MyDialog()
    dialog.show()
    sys.exit(app.exec_())

