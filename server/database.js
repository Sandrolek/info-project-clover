var sqlite3 = require("sqlite3").verbose();
var md5 = require("md5");

const DBSOURCE = "db.sqlite";


// Создаём таблицы, если их ещё нет в базе данных
let db = new sqlite3.Database(DBSOURCE, (err) => {
    
    if (err) {
        // Если случилаь ошибка, выводим её в консоль сервера
        console.error(err.message);
        throw err;
    }

    sqlReports = `
        CREATE TABLE IF NOT EXISTS reports (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            datetime TIMESTAMP
        )
    `;

    sqlLogs = `
        CREATE TABLE IF NOT EXISTS logs (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            coordinates TEXT,
            state VARCHAR(4),
            report_id INTEGER,
            FOREIGN KEY (report_id) REFERENCES reports (id)
        )
    `
    
    db.run(sqlReports)
    db.run(sqlLogs)

});

module.exports = db;
