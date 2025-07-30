from telegram import Update
from telegram.ext import ApplicationBuilder, CommandHandler
import asyncio
import json
import queue


# Global message queue for cross-thread communication
message_queue = queue.Queue()
# Store the most recent chat_id
last_chat_id = None


def decode_json(path):
    # Function to decode JSON file to find username and API_KEY
    with open(path, 'r') as file:
        data = json.load(file)
        API_KEY = data["API_KEY"]
    return API_KEY


async def start_command(update, context):
    # For the bot to know the user after starting
    global last_chat_id
    print("Start")
    name = update.effective_user.first_name
    last_chat_id = update.effective_chat.id
    text = f'Hello {name}! Welcome to FALCON\'s GUI Companion Bot. Any unexpected values will be reported here.'
    await context.bot.send_message(chat_id=last_chat_id, text=text)


def send_msg(update, context, message):
    # Function to send message to user
    text = f"Unexpected values: {message}"
    update.message.reply_text(text)


def main():
    print("Bot started")
    API_KEY = decode_json('Frontend/TelegramBot/creds.json')
    application = ApplicationBuilder().token(API_KEY).build()
    start_handler = CommandHandler('start', start_command)
    application.add_handler(start_handler)

    application.run_polling()


async def botloop_routine(API_KEY):
    print("Bot started")
    application = ApplicationBuilder().token(API_KEY).build()
    application.add_handler(CommandHandler('start', start_command))

    await application.initialize()
    await application.updater.start_polling()
    await application.start()
    try:
        while True:
            # Check the message queue for outgoing messages
            try:
                item = message_queue.get_nowait()
                if isinstance(item, tuple) and len(item) == 2:
                    chat_id, message = item
                await application.bot.send_message(chat_id=item[0], text=item[1])
            except queue.Empty:
                pass
            await asyncio.sleep(1)
    finally:
        await application.updater.stop()
        await application.stop()
        await application.shutdown()


async def botloop_start(API_KEY):
    bot_routine = asyncio.create_task(botloop_routine(API_KEY))
    await bot_routine


def botloop():
    # Start the bot loop
    API_KEY = decode_json('Frontend/TelegramBot/creds.json')
    asyncio.run(botloop_start(API_KEY))


async def send_msg_ext(message, chat_id=None):
    global last_chat_id
    # Use the provided chat_id, or fall back to the last seen one
    if chat_id is None:
        chat_id = last_chat_id
    if chat_id is not None:
        message_queue.put((chat_id, message))
    else:
        print("No chat_id available to send message.")


if __name__ == '__main__':
    main()
