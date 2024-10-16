# Importing Libraries
import serial
import time
import pandas as pd
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error
import numpy as np
import matplotlib.pyplot as plt  # To visualize

# save data using pickle: https://www.geeksforgeeks.org/how-to-use-pickle-to-save-and-load-variables-in-python/

# connect to arduino and clear buffer
ser = serial.Serial(port='COM4', baudrate=115200)


# create empty array for data storage
d = []
x = str(0)  #was this causing plate to swing up initially????
ser.write(x.encode())
time.sleep(2)
ser.flush()

# vary the bend angle from 0 to 60 degrees in 10 degree increments.
# python sends a value, say 1, and arduino reads that value, multiplies the value by 10 and then subrracts 10.
# so 1 would mean 0 degrees, 2 -> 10 degrees, etc
# 7 equals to about 0 degrees pitch for IMU
#k = [7, 6, 5, 4, 3, 2, 1]
k = [0, 1, 2, 3, 4, 5, 6, 5, 4, 3, 2, 1, 0]

# collect data down and up 5 times
for _ in range(5):
    for n in k:
        x = str(n)
        ser.write(x.encode())

        # This loop collects p amount of data per each angle
        p = 50
        for _ in range(p):
            data = ser.readline().strip().decode()
            # Attempt to split data by tabs and spaces, then take only the relevant parts
            parts = data.split()
            # Ensure exactly 3 parts for each row
            if len(parts) != 4 and parts[0] != '0':
                continue  # Skip this iteration if data is not as expected
            # Convert parts to floats, replace with 100 if conversion fails
            float_values = []
            for m in parts:
                try:
                    float_values.append(float(m))
                except ValueError:
                    float_values.append(100)  # Default value if conversion fails
            print(float_values)
            d.append(float_values)  # Append the row of floats to list d

ser.close()

# convert list d to pandas data frame and save as csv
df = pd.DataFrame(d)
df.columns = ['Theoretical Angle (deg)', 'IMU Angle (deg)', 'ADC Value', 'Rotary Encoder']
#df.replace([np.inf, -np.inf], np.nan, inplace=True)
#df.dropna(how="all", inplace=True)

df.to_csv('Bending_data_10_14_2024_v12_test.csv', index=False)
print(df)


x = np.array(df['Theoretical Angle (deg)']).reshape(-1, 1)
y = np.array(df['IMU Angle (deg)']).reshape(-1, 1)
# regression analysis for  future comparison to current data
# fit the model
model = LinearRegression()
model.fit(x, y)
y_pred = model.predict(x)
#r2_score = model.score(x, y)
#print(f"R-squared value: {r2_score}")

# Evaluate mse and pearson correlation of adafruit IMU vs Khanna Lab IMU
mse = mean_squared_error(x, y)
correlation = df['Theoretical Angle (deg)'].corr(df['IMU Angle (deg)'])
print(f"mean squared error of IMU's: {mse}")
print(f"correlation of IMU's: {correlation}")

# plot raw Adafruit IMU data vs Khanna Lab IMU
length_of_x = len(x)
# Create an array with values from 1 to length_of_x
value = np.arange(1, length_of_x + 1).reshape(-1, 1)
plt.scatter(value, x, color='blue', label='Theoretical IMU')
plt.scatter(value, y, color='red', label='Adafruit IMU')
# plt.plot(x, y_pred, color='red') # in case we want to plot best fit line for future use
# Add labels and title
plt.xlabel('Position in Array (will do timestamp later)')  # Label for the x-axis
plt.ylabel('Angle (deg)')  # Label for the y-axis
plt.title('Theoretical IMU vs. Adafruit IMU')  # Title of the plot

# Add a legend
plt.legend()
plt.show()



