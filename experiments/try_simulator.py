"""Schnell-Test des SimulatorDrivers."""

from hexapod.drivers.simulator import SimulatorDriver

# Test 1: Normaler Gebrauch
with SimulatorDriver(num_channels=4, verbose=True) as driver:
    driver.set_position(0, 1500)
    driver.set_position(1, 1700)
    driver.set_positions({2: 1800, 3: 1200})
    print("Pos 0:", driver.get_position(0))
    print("Snapshot:", driver.snapshot())
    driver.disable(0)
    print("Nach disable:", driver.snapshot())

print("Closed:", driver.is_closed)

# Test 2: Validierung
print("--- Validierung ---")
with SimulatorDriver(num_channels=4) as driver:
    try:
        driver.set_position(0, 100)
    except ValueError as e:
        print("Erwartet:", e)
    try:
        driver.set_position(5, 1500)
    except ValueError as e:
        print("Erwartet:", e)
