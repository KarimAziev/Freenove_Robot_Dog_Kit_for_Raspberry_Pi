class Kalman_Filter:
    def __init__(self, Q: float, R: float) -> None:
        """
        Initializes the Kalman Filter with the provided process noise (Q) and measurement noise (R).

        Args:
            Q (float): Process noise covariance.
            R (float): Measurement noise covariance.
        """
        # Process noise covariance
        self.Q = Q
        # Measurement noise covariance
        self.R = R
        # A posteriori error estimate covariance (initialized to 1)
        self.P_k_k1 = 1.0
        # Kalman gain
        self.Kg = 0.0
        # A priori error estimate covariance (initialized to 1)
        self.P_k1_k1 = 1.0
        # A priori estimate (initially 0)
        self.x_k_k1 = 0.0
        # Previous ADC value
        self.ADC_OLD_Value = 0.0
        # Latest measurement/observation (initialized to 0)
        self.Z_k = 0.0
        # Previous filtered ADC value
        self.kalman_adc_old = 0.0

    def kalman(self, ADC_Value: float) -> float:
        """
        Performs one iteration of the Kalman filter given a new ADC value.

        Args:
            ADC_Value (float): The current ADC measurement.

        Returns:
            float: The updated Kalman filtered ADC value after this iteration.
        """
        self.Z_k = ADC_Value

        # Large difference detection and initial estimate
        if abs(self.kalman_adc_old - ADC_Value) >= 60:
            # Blend new ADC value with the previous filtered value
            self.x_k1_k1 = ADC_Value * 0.400 + self.kalman_adc_old * 0.600
        else:
            # Maintain old filtered estimate if changes are small
            self.x_k1_k1 = self.kalman_adc_old

        # State prediction (no control input assumed, so it's just the prior state)
        self.x_k_k1 = self.x_k1_k1

        # Error covariance prediction
        self.P_k_k1 = self.P_k1_k1 + self.Q

        # Compute Kalman Gain
        self.Kg = self.P_k_k1 / (self.P_k_k1 + self.R)

        # State update with the Kalman Gain
        kalman_adc = self.x_k_k1 + self.Kg * (self.Z_k - self.kalman_adc_old)

        # Update the estimate covariance based on Kalman Gain
        self.P_k1_k1 = (1 - self.Kg) * self.P_k_k1
        self.P_k_k1 = self.P_k1_k1

        # Update the previous Kalman-filtered ADC value
        self.kalman_adc_old = kalman_adc

        return kalman_adc