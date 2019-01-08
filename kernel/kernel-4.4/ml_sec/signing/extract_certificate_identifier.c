#include <stdlib.h>
#include <stdio.h>
#include <openssl/x509.h>
#include <openssl/bio.h>
#include <openssl/pem.h>
#include <openssl/asn1.h>
#include <openssl/err.h>

int main(int argc, char *argv[])
{
	BIO *certificate_bio   = NULL;
	X509 *certificate_x509 = NULL;
	int ret  = 0;
	size_t i = 0;

	size_t issuer_start_idx;
	const unsigned char *issuer_bytes_start;
	long issuer_bytes_len;
	int tag, xclass;
	int asn1_ret;

	ASN1_INTEGER *serial;
	X509_NAME *issuer;

	if (argc != 2) {
		printf("Usage: %s <certificate_file_path>\n", argv[0]);
		ret = 1;
		goto exit;
	}

	OpenSSL_add_all_algorithms();
	ERR_load_BIO_strings();
	ERR_load_crypto_strings();

	certificate_bio = BIO_new(BIO_s_file());
	ret = BIO_read_filename(certificate_bio, argv[1]);
	if (!ret) {
		printf("\nerror: failed to read filename\n");
		ret = 1;
		goto exit;
	}

	certificate_x509 = PEM_read_bio_X509(certificate_bio, NULL, 0, NULL);
	if (!certificate_x509) {
		printf("\nerror: error loading certificate into memory\n");
		ret = 2;
		goto exit;
	}

	/* Get serial number */
	serial = X509_get_serialNumber(certificate_x509);
	if (serial->length == 0) {
		printf("\nerror: empty serial number");
		ret = 3;
		goto exit;
	}

	/* Get issuer name */
	issuer = X509_get_issuer_name(certificate_x509);

	if (issuer == NULL) {
		printf("\nerror: failed to extract issuer name");
		ret = 4;
		goto exit;
	}
	if (issuer->bytes->length == 0) {
		printf("\nerror: empty issuer name");
		ret = 5;
		goto exit;
	}

	/* Parse the issuer "sequence" header so we can skip it */
	issuer_bytes_start = (const unsigned char *)issuer->bytes->data;
	asn1_ret = ASN1_get_object(&issuer_bytes_start,
				   &issuer_bytes_len,
				   &tag,
				   &xclass,
				   issuer->bytes->length);

	if (asn1_ret & 0x80) {
		printf("\nerror: ASN1_get_object");
		ret = 6;
		goto exit;
	}

	if (tag != V_ASN1_SEQUENCE) {
		printf("\nerror: unexpected issuer head tag");
		ret = 7;
		goto exit;
	}

	issuer_start_idx = issuer_bytes_start - (const unsigned char *)issuer->bytes->data;

	/* Print serial leading zero if needed */
	if ((signed char)(serial->data[i]) < 0)
		printf("00");

	/* Print serial */
	for (i = 0; i < serial->length; i++)
		printf("%02x", serial->data[i]);

	/* Print issuer name */
	for (i = issuer_start_idx; i < issuer->bytes->length; i++)
		printf("%02x", (unsigned char)issuer->bytes->data[i]);


exit:
	if (certificate_x509 != NULL)
		X509_free(certificate_x509);

	if (certificate_bio != NULL)
		BIO_free_all(certificate_bio);

	return ret;
}





