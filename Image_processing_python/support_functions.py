#Maps input value to new desired range
def MapValue(value, lowInput, highInput, lowOutput, highOutput):   
    return int((value - lowInput) * (highOutput - lowOutput) / (highInput - lowInput) + lowOutput)