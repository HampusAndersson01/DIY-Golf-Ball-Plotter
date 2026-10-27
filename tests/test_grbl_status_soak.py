from scripts.grbl_status_soak import GrblStatusParser


def test_grbl_status_parser_returns_only_complete_status_reports():
    parser = GrblStatusParser()

    assert parser.feed(b"Grbl 1.1h ['$' for help]\r\n<Id") == []
    assert parser.feed(b"le|WPos:0.000,0.000,0.000>\r\nok\r\n") == [
        "<Idle|WPos:0.000,0.000,0.000>"
    ]

