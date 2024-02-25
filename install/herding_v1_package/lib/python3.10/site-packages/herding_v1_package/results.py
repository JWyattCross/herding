import matplotlib.pyplot as plt
import numpy as np

def Results(simulation):
    tracking_error = np.linalg.norm(simulation.agent.tracking_error, axis=0)

    # Plotting
    plt.figure(figsize=(10, 6))
    plt.plot(tracking_error, label='Tracking Error')
    plt.xlabel('Time Step')
    plt.ylabel('Tracking Error Magnitude')
    plt.title('Tracking Error Over Time')
    plt.legend()
    plt.grid(True)
    plt.show()