var express = require("express");
var router = express.Router();
var db = require("../database");

router.post("/logs", function (req, res, next) {
    const data = req.body;

    const datetime = data["datetime"];
    const logs = data["logs"];

    // Insert report
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

    function insertLogs(reportId) {
        // Insert logs
        for (log of logs) {
            const LogQuery =
                "INSERT INTO logs (coordinates, state, report_id) VALUES (?, ?, ?)";
            const LogParams = [
                JSON.stringify(log["coordinates"]),
                log["state"],
                reportId,
            ];

            db.run(LogQuery, LogParams, function (err, result) {
                if (err) {
                    res.status(400).json({ error: err.message });
                    return;
                }
            });
        }

        res.json({
            status: 200,
            id: reportId,
        });
    }
});

router.get("/logs/all", function (req, res, next) {
    const sql = "SELECT * FROM logs";

    db.all(sql, [], function (err, rows) {
        if (err) {
            res.status(400).json({ error: err.message });
            return;
        }

        res.json({
            status: 200,
            data: rows,
        });
    });
});

router.get("/reports/all", function (req, res, next) {
    const sql = "SELECT * FROM reports";

    db.all(sql, [], function (err, rows) {
        if (err) {
            res.status(400).json({ error: err.message });
            return;
        }

        res.json({
            status: 200,
            data: rows,
        });
    });
});

router.get("/logs", function (req, res, next) {
    const sql = `
        SELECT logs.*, reports.datetime
        FROM logs
        JOIN reports ON logs.report_id = reports.id
        WHERE logs.report_id = (SELECT id FROM reports ORDER BY datetime DESC LIMIT 1);    
    `;

    db.all(sql, [], function (err, rows) {
        if (err) {
            res.status(400).json({ error: err.message });
            return;
        }

        let datetime = null

        rows.forEach((row) => {
            datetime = row.datetime
            delete row.datetime
        });

        res.json({
            status: 200,
            datetime,
            data: rows,
        });
    });
});

module.exports = router;
