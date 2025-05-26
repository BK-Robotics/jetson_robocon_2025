import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from mpl_toolkits.mplot3d import Axes3D

# Data input
velocity = np.array([
    50, 50, 50, 60, 60, 60, 60, 60, 70, 70, 70, 70, 70, 80, 80, 90, 90,
    50, 50, 50, 50, 60, 60, 60, 50, 50, 50, 60, 60, 60, 60
])
angle = np.array([
    30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30,
    45, 45, 45, 45, 45, 45, 45, 61.5, 61.5, 61.5, 61.5, 61.5, 61.5, 61.5
])
distance = np.array([
    7900, 7900, 7900, 9800, 9700, 9650, 9700, 9750, 10520, 10480, 10470, 10900, 10700, 11800, 12200, 12700, 12900,
    7750, 7800, 7800, 7850, 7400, 7400, 7500, 6900, 6900, 6800, 8000, 8000, 8000, 8000
])

# Prepare input features for regression
X = np.column_stack((velocity, angle))
y = distance

# Fit linear regression model
model = LinearRegression()
model.fit(X, y)
predicted = model.predict(X)

# Print detailed regression information
print("\n===== LINEAR REGRESSION MODEL INFORMATION =====")
print(f"Intercept (β₀): {model.intercept_:.2f}")
print(f"Velocity Coefficient (β₁): {model.coef_[0]:.2f}")
print(f"Angle Coefficient (β₂): {model.coef_[1]:.2f}")
print(f"R² Score: {model.score(X, y):.4f}")

# Calculate Mean Squared Error
mse = np.mean((predicted - distance) ** 2)
print(f"Mean Squared Error: {mse:.2f}")
print(f"Root Mean Squared Error: {np.sqrt(mse):.2f}")

# Regression equation
print("\n===== REGRESSION EQUATION =====")
print(f"Distance = {model.intercept_:.2f} + {model.coef_[0]:.2f} × Velocity + ({model.coef_[1]:.2f}) × Angle")

# Calculate statistics for actual vs predicted
mean_actual = np.mean(distance)
mean_predicted = np.mean(predicted)
print(f"\nMean actual distance: {mean_actual:.2f} mm")
print(f"Mean predicted distance: {mean_predicted:.2f} mm")

# Create 3D visualization of the regression plane
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# Scatter plot of actual data
scatter = ax.scatter(velocity, angle, distance, c='blue', marker='o', s=50, 
                    alpha=0.7, label='Actual Data')

# Create a mesh grid for the regression plane
vel_range = np.linspace(min(velocity), max(velocity), 20)
ang_range = np.linspace(min(angle), max(angle), 20)
V, A = np.meshgrid(vel_range, ang_range)
Z = model.intercept_ + model.coef_[0] * V + model.coef_[1] * A

# Plot the regression plane
surface = ax.plot_surface(V, A, Z, alpha=0.4, color='red', 
                        rstride=1, cstride=1, linewidth=0, 
                        antialiased=True)

# Enhance visual appearance
ax.set_xlabel('Velocity', fontsize=12, labelpad=10)
ax.set_ylabel('Angle (degrees)', fontsize=12, labelpad=10)
ax.set_zlabel('Distance (mm)', fontsize=12, labelpad=10)
ax.set_title('3D Linear Regression Model', fontsize=14, pad=20)

# Add a legend
ax.legend(loc='upper left')

# Add grid and improve visibility
ax.grid(True)
ax.view_init(elev=30, azim=45)  # Adjust viewing angle

# Add annotation with equation
equation_text = f"Distance = {model.intercept_:.2f} + {model.coef_[0]:.2f}×Velocity + ({model.coef_[1]:.2f})×Angle"
ax.text2D(0.05, 0.95, equation_text, transform=ax.transAxes, fontsize=12, 
          bbox=dict(facecolor='white', alpha=0.7))

plt.tight_layout()
plt.show()