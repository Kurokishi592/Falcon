from telegram import Update
from telegram.ext import ApplicationBuilder, CommandHandler
import json

def decode_json(path):
    # Function to decode JSON file to find username and API_KEY
    with open(path, 'r') as file:
        data = json.load(file)
        username = data["username"]
        API_KEY = data["API_KEY"]
    return username, API_KEY


async def start_command(update, context):
    # For the bot to know the user after starting
    print("Start")
    name = update.effective_user.first_name
    text = f'Hello {name}! Welcome to FALCON\'s GUI Companion Bot. Any unexpected values will be reported here.'
    await context.bot.send_message(chat_id=update.effective_chat.id, text=text)


def send_msg(update, context, username, message):
    # Function to send message to user
    text = f"Unexpected values: {message}"
    update.message.reply_text(text)


def main():
    username, API_KEY = decode_json('Frontend/TelegramBot/creds.json')
    application = ApplicationBuilder().token(API_KEY).build()
    start_handler = CommandHandler('start', start_command)
    application.add_handler(start_handler)

    application.run_polling()


if __name__ == '__main__':
    main()
