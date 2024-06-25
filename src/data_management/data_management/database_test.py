import sqlite3

connection = sqlite3.connect("test.db")

# Create a cursor (used to execute SQL commands)
cursor = connection.cursor()

cursor.execute("SELECT species FROM fish")
species_list = cursor.fetchall()
print(species_list)
