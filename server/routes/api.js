var express = require("express");
var router = express.Router();
var db = require("../database");


// Записывает логи, полученные от дрона
router.post("/logs", function (req, res, next) {
    const data = req.body;

    const datetime = data["datetime"];
    const logs = data["logs"];

    const ReportQuery = "INSERT INTO reports (datetime) VALUES (?)";
    const ReportParams = [datetime];

    let reportId = null;

    db.run(ReportQuery, ReportParams, function (err, result) {
        if (err) {
            res.status(400).json({ error: err.message });
            return;
        }

        reportId = this.lastID;

        insertLogs(reportId);
    });

    // После записи нового репорта, записываются все связанные с ним логи
    function insertLogs(reportId) {
        for (log of logs) {
            const LogQuery =
                "INSERT INTO logs (coordinates, state, report_id) VALUES (?, ?, ?)";
            const LogParams = [
                JSON.stringify(log["coordinates"]),
                log["state"],
                reportId,
            ];

            db.run(LogQuery, LogParams, function (err, result) {
                // Если произошла ошибка, выводим её
                if (err) {
                    res.status(400).json({ error: err.message });
                    return;
                }
            });
        }

        // После успешной записи отправляем статус 200
        res.json({
            status: 200,
            id: reportId,
        });
    }
});

// Endpoint для получение последниг логов
router.get("/logs", function (req, res, next) {
    const sql = `
        SELECT logs.*, reports.datetime
        FROM logs
        JOIN reports ON logs.report_id = reports.id
        WHERE logs.report_id = (SELECT id FROM reports ORDER BY datetime DESC LIMIT 1);    
    `;

    // Получаем логи
    db.all(sql, [], function (err, rows) {
        // Если случилась ошибка, выводим её
        if (err) {
            res.status(400).json({ error: err.message });
            return;
        }

        let datetime = null

        // Форматируем вывод
        rows.forEach((row) => {
            datetime = row.datetime
            delete row.datetime
        });

        // Отправляем логи на клиент
        res.json({
            status: 200,
            datetime,
            data: rows,
        });
    });
});

module.exports = router;
