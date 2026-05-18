from app import create_app

app = create_app()

if __name__ == "__main__":
    logger = app.logger
    logger.info("Starting Flask backend process")
    try:
        app.run(
            host=app.config["FLASK_HOST"],
            port=app.config["FLASK_PORT"],
            debug=app.config["FLASK_DEBUG"],
        )
    except Exception:
        logger.exception("Flask backend crashed during app.run")
        raise
    finally:
        logger.info("Flask backend process exiting")
