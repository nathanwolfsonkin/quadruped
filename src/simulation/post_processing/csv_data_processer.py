import csv

class CSVDataProcesser():
    def __init__(self):
        pass
    
    @staticmethod
    def csv_to_dict(file_path):
        with open(file_path, mode='r', newline='', encoding='utf-8') as file:
            reader = list(csv.reader(file))  # Convert reader to a list to access columns easily
            
            if not reader:
                return {}
            
            # Transpose rows to columns
            columns = list(zip(*reader))
            
            # Create dictionary with first element as key and remaining as list, converting to double
            data_dict = {}
            for col in columns:
                key = col[0]
                values = []
                for value in col[1:]:
                    try:
                        values.append(float(value))
                    except ValueError:
                        values.append(value)  # Keep original if conversion fails
                data_dict[key] = values
            
        return data_dict
