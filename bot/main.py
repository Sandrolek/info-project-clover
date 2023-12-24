import logging
import requests
from aiogram import Bot, Dispatcher, types, executor
from aiogram.types import InlineKeyboardMarkup, InlineKeyboardButton, CallbackQuery, Message, ReplyKeyboardMarkup, KeyboardButton
import json
import os
import sys

# Configure logging
logging.basicConfig(level=logging.INFO)

API_TOKEN = '6630879357:AAHjhfaSuqt1HOyC1JXJnEIKzRnT2LzO7Lc'
URL = 'https://drone-stats.onrender.com/api/logs'

# Инициализировуем бота и диспетчера
bot = Bot(token=API_TOKEN)
dp = Dispatcher(bot)

# Обработчик нажатия на кнопку "Получить отчёт"
@dp.message_handler(lambda message: message.text == 'Получить отчёт', state='*')
async def get_last_report(message: types.Message):
    await update_command(message)

# Преветственное окно
@dp.message_handler(commands=['start'])
async def send_welcome(message: types.Message):
    # Определение клавиатуры
    keyboard = ReplyKeyboardMarkup(resize_keyboard=True)
    button = KeyboardButton('Получить отчёт')
    keyboard.add(button)

    await message.answer("Привет! Я бот. Нажми на кнопку 'Получить отчёт' для получения последнего отчёта.", reply_markup=keyboard)


# Определяем команду /last для бота, которая будет получать последний репорт с сервера
@dp.message_handler(commands=['last'])
async def update_command(message: Message):
    try:
        # получаем с сервера необходимые данные в формате json
        data = requests.get(URL).json()
        
        # дата последнего репорта
        datetime = data['datetime']

        # список логов принадлежащих 1 репорту
        logs = [
            f"Coordinates: {log['coordinates'][1:-1]}\nState: {log['state']}" for log in data['data']
        ]
      
        # если есть логи, то выводим
        if len(data['data']) > 0:            
            await message.reply(f'Report - {datetime}\n\n' + '\n\n'.join(logs))
        else:
            await message.reply("No data available.")
            
    except Exception as e:
        await message.reply(f"Error fetching data: {e}")

# Запускаем бота
if __name__ == '__main__':
    executor.start_polling(dp)
