import re

def isNumber(value):
    # Remove F and U suffixes from numeric values
    newValue = str(value).replace('F', '').replace('U', '')
    try:
        eval(newValue)
        return newValue
    except ValueError:
        return value

def parse_h_file(file_path):
    constants = {}
    with open(file_path, 'r') as file:
        for line in file:
            match = re.match(r'#define\s+(\w+)\s+(.+)', line)
            if match:
                key, value = match.groups()
                constants[key] = value
    return constants

# Usage
constants = parse_h_file('../config.h')
print(constants)