import csv
from json import load, dump

def read_json(json_path : str) -> dict :
    """read data in .JSON file

    Args:
        json_path (str): path of the json file.

    Returns:
        dict: data inside json file
    """
    with open(json_path, 'r') as read_json :
        json_data = load(read_json)
        read_json.close()
    return json_data

def write_json(data : dict, json_path : str) :
    """write or create .JSON file.

    Args:
        data (dict): data for write in json file. 
        json_path (str): path of the json file.
    """
    with open(json_path, 'w') as write_json :
        dump(data, write_json, indent = 4)
        write_json.close()
        
def update_json(new_data_dic : dict, json_path : str) :
    """insert new dict to .JSON file.

    Args:
        new_data_dic (dict): dict of data to insert.
        json_path (str): path of the json file.
    """
    file_data = read_json(json_path)
    for each_dic in new_data_dic :
        file_data[each_dic] = new_data_dic[each_dic]
        write_json(file_data, json_path)
        
def read_csv(csv_path : str) -> list :
    """read data in .CSV file.

    Args:
        csv_path (str): path of the csv file.

    Returns:
        list: data inside csv file.
    """
    with open(csv_path, 'r') as read_obj :
        csv_data = csv.reader(read_obj)
        list_of_csv = list(csv_data)
        read_obj.close()
    return list_of_csv

def write_csv(data : list, csv_path : str) :
    """write or create .CSV file.

    Args:
        data (list): data for write in csv file. 
        csv_path (str): path of the csv file.
    """
    with open(csv_path, 'w') as write_obj :
        csv_writer = csv.writer(write_obj, lineterminator = '\n')
        csv_writer.writerows(data)
        write_obj.close()
        
def update_csv(new_csv_data : list, csv_path : str) :
    """insert data to .CSV file.

    Args:
        new_csv_data (list): list of data to insert.
        csv_path (str): path of the csv file.
    """
    csv_data = read_csv(csv_path)
    csv_data.append(new_csv_data)
    with open(csv_path, 'w') as write_obj :
        csv_writer = csv.writer(write_obj, lineterminator = '\n')
        csv_writer.writerows(csv_data)
        write_obj.close()
    csv_data = []
    new_csv_data = []