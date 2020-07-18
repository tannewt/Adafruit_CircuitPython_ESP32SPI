class SSLContext:
    def __init__(self, *, client_certificate=None, private_key=None):
            def set_certificate(self, ):
        """Sets client certificate. Must be called
        BEFORE a network connection is established.
        :param str client_certificate: User-provided .PEM certificate up to 1300 bytes.
        """
        # TODO: Ask Brent why this is.
        if self.status == WL_CONNECTED:
            raise RuntimeError(
                "set_certificate must be called BEFORE a connection is established."
            )

        if client_certificate:
            if isinstance(client_certificate, str):
                client_certificate = bytes(client_certificate, "utf-8")
            if not client_certificate.startswith(b"-----BEGIN CERTIFICATE"):
                raise ValueError("client certificate must start with -----BEGIN CERTIFICATE")
            if len(client_certificate) > 1300:
                raise ValueError("client_certificate must be less than 1300 bytes")
            self._client_certificate = client_certificate

        if private_key:
            if isinstance(private_key, str):
                private_key = bytes(private_key, "utf-8")
            if not private_key.startswith("-----BEGIN RSA"):
                raise ValueError("private_key must start with -----BEGIN RSA")
            if len(private_key) > 1700:
                raise ValueError("private_key must be less than 1700 bytes")
            self._private_key = private_key

        # This is an xor that ensure they are either both set or neither set.
        if private_key is None != client_certificate is None:
            raise ValueError("Must provide both client_certificate and private_key or neither")

    def wrap_socket(sock):
        resp = sock.esp.send_command_get_response(_SET_CLI_CERT, (client_certificate,))
        if resp[0][0] != 1:
            raise RuntimeError("Failed to set client certificate")
        resp = sock.esp.send_command_get_response(_SET_PK, (private_key,))
        if resp[0][0] != 1:
            raise RuntimeError("Failed to set private key.")
        return sock

def create_default_context():
    return SSLContext()
