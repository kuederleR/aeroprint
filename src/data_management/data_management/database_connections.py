import sqlite3
import os

class db_connection():
    def __init__(self, connection_name):
        self.abs_path = os.path.dirname(os.path.abspath(__file__))
        self.datasets_path = self.abs_path + "/datasets/"
        self.connection_name = connection_name

    def print_path(self):
        print(self.datasets_path)

def main():
    dbc = db_connection("DEEZ_NUTS")
    dbc.print_path()

if __name__ == "__main__":
    main()
