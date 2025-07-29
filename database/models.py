import sqlite3

def create_tables(db_path='data.db'):
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    