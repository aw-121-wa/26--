from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VISION_API = ROOT / "App" / "vision" / "vision_api.c"


def main():
    source = VISION_API.read_text(encoding="utf-8")

    assert "#define VISION_VOFA_UART        (&huart2)" in source
    assert 'vofa_send_values("vision_tx", values, 4u);' in source
    assert 'vofa_send_values("vision_rx", values, 5u);' in source
    assert "HAL_UART_Transmit(VISION_VOFA_UART" in source
    assert "vofa_send_request(sequence, payload);" in source
    assert "vofa_send_result(&pending_result);" in source
    assert "request_tx_count++;" in source

    transmit_ok = source.index("if (HAL_UART_Transmit(&huart5")
    request_log = source.index("vofa_send_request(sequence, payload);")
    assert transmit_ok < request_log

    result_sequence = source.index("pending_result.sequence = parser.sequence;")
    result_log = source.index("vofa_send_result(&pending_result);")
    result_pending = source.index("result_pending = 1;", result_sequence)
    assert result_sequence < result_log < result_pending

    assert "uint16_t right_request_count;" in source
    assert "right_request_count = request_tx_count;" in source
    assert "if (status != VISION_STATUS_OK && request_tx_count == right_request_count)" in source


if __name__ == "__main__":
    main()
