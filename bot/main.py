import logging
import requests
from aiogram import Bot, Dispatcher, types
from aiogram.types import InlineKeyboardMarkup, InlineKeyboardButton, CallbackQuery, Message
import json
import os
import sys

API_TOKEN = '6630879357:AAHjhfaSuqt1HOyC1JXJnEIKzRnT2LzO7Lc'
URL = 'https://drone-stats.onrender.com/api/logs'

# Configure logging
logging.basicConfig(level=logging.INFO)

# Initialize bot and dispatcher
bot = Bot(token=API_TOKEN)
dp = Dispatcher(bot)

# Define the start command handler
@dp.message_handler(commands=['start'])
async def send_welcome(message: types.Message):
    keyboard = InlineKeyboardMarkup()
    update_button = InlineKeyboardButton("Last log", callback_data='last')
    all_logs_button = InlineKeyboardButton("All logs", callback_data='all')
    keyboard.add(update_button, all_logs_button)
    await message.reply("Welcome! Press the 'Update' button to fetch the last log or 'All logs' to get all logs.", reply_markup=keyboard)

# Define the /update command handler
@dp.message_handler(commands=['last'])
async def update_command(message: Message):
    try:
        response = requests.get(URL)
        data = response.json()

        if len(data['data']) > 0:
            last_log = data['data'][-1]
            coords = [coord.strip() for coord in last_log['coordinates'].replace('[', '').replace(']', '').split(',')]

            formatted_message = f"❗️ Report ❗️\nDatetime: {last_log['datetime']}\nCoordinates: {coords[0]}° N {coords[1]}° E\nStatus: {last_log['status']}"
            await message.reply(formatted_message)
        else:
            await message.reply("No data available.")
    except Exception as e:
        await message.reply(f"Error fetching data: {e}")

# Define the /all command handler
@dp.message_handler(commands=['all'])
async def all_logs_command(message: Message):
    try:
        response = requests.get(URL)
        data = response.json()

        if len(data['data']) > 0:
            all_logs = [f"❗️ Report - {log['id']} ❗️\nDatetime: {log['datetime']}\nCoordinates: { ' '.join([ coord + '°' for coord in log['coordinates'].replace('[', '').replace(']', '').split(',')]) }\nStatus: {log['status']}\n" for log in data['data']]
            formatted_message = '\n'.join(all_logs)
            await message.reply(formatted_message)
        else:
            await message.reply("No data available.")
    except Exception as e:
        await message.reply(f"Error fetching data: {e}")

# Define the /restart command handler
@dp.message_handler(commands=['restart'])
async def restart_command(message: Message):
    await message.reply("Restarting the bot...")

    # Stop the current instance of the bot
    os.execl(sys.executable, sys.executable, *sys.argv)

# Start the bot
if __name__ == '__main__':
    from aiogram import executor
    executor.start_polling(dp, skip_updates=True)
