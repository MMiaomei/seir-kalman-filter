# Estimate parameters of SEIR model using an unscented Kalman filter and state data.

1. Use the SEIR model from [CoronaTracker: World-wide COVID-19 Outbreak Data Analysis and Prediction](https://www.who.int/bulletin/online_first/20-255695.pdf)

2. Fix two model parameters from that paper as:
  - incubation rate, sigma = 1 / (5.2 days)
  - recovery rate, gamma = 1 / (2.3 days)
  
3. Estimate model variables (`S,E,I,R`) and model parameter `beta`, which controls the rate of spread.

4. The observed data is a state's number of infected patients each day. See [Jupyter notebook](data/GetStateData.ipynb).

5. Use an unscented Kalman filter with default parameter settings.

6. After filtering, predict for 30 days into the future.

7. Identify the date with peak infections.

## Notes

- Define `N` as state's population (from Wikipedia)
- The model breaks down at long time predictions (i.e., populations < 0).

## Example output

![alt text](/imgs/example_output.png)
