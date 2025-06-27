def find_init_soc(voltage, voltage_list, soc_list):
    """
    Given an initial voltage measurement, use binary search and interpolation to find the corresponding initial SoC.

    Parameters:
    voltage (float): The initial voltage measurement.
    voltage_list (list): The list of voltage values from the lookup table.
    soc_list (list): The list of SoC values corresponding to the voltage list.

    Returns:
    float: The interpolated initial SoC value.
    """
    # Binary search to find the two closest voltages
    left, right = 0, len(voltage_list) - 1
    while left < right:
        mid = (left + right) // 2
        if voltage_list[mid] > voltage:
            left = mid + 1
        else:
            right = mid

    # Now, left is the index of the voltage just greater than or equal to the given voltage
    if left == 0:
        return soc_list[0]  # If voltage is higher than the highest voltage, return the first SoC
    elif left == len(voltage_list):
        return soc_list[-1]  # If voltage is lower than the lowest voltage, return the last SoC
    else:
        # Interpolate between the found values
        v1, v2 = voltage_list[left - 1], voltage_list[left]
        s1, s2 = soc_list[left - 1], soc_list[left]
        
        # Linear interpolation formula
        return s1 + (s2 - s1) * (voltage - v1) / (v2 - v1)


# Lookup table
voltage_list = [4.20, 4.16, 4.12, 4.10, 4.08, 4.06, 4.04, 4.02, 4.00, 3.80, 3.20, 2.50]
soc_list = [1.0, 0.91, 0.82, 0.73, 0.64, 0.55, 0.46, 0.36, 0.27, 0.18, 0.09, 0.0]

# Test example: Finding the init SoC for a given voltage (e.g., 4.05)
voltage = 4.15
init_soc = find_init_soc(voltage, voltage_list, soc_list)
print(f"Initial SoC for voltage {voltage}V: {init_soc*100:.2f}%")
