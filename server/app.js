var express = require("express");
var path = require("path");
var cookieParser = require("cookie-parser");
var logger = require("morgan");
var cors = require('cors')

var indexRouter = require("./routes/index");
var apiRouter = require("./routes/api");

var app = express();

// Настройка CORS
app.use((req, res, next) => {
    res.header("Access-Control-Allow-Origin", "*");
    res.header(
        "Access-Control-Allow-Headers",
        "Origin, X-Requested-With, Content-Type, Accept"
    );
    next();
});

app.options('*', cors()) // include before other routes
app.use(cors());

app.use(logger("dev"));
app.use(express.json());
app.use(express.urlencoded({ extended: false }));
app.use(cookieParser());
app.use(express.static(path.join(__dirname, "public")));

// Указываем наши route'ы
app.use("/", indexRouter);
app.use("/api", apiRouter);

module.exports = app;
