function dutyCycle = convert_to_duty_cycle(angle)
    degreesInMs = 8.3333;
    dutyCycle = 1000 + angle * degreesInMs;
end