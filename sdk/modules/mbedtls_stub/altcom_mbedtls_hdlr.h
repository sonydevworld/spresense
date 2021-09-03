/****************************************************************************
 * modules/mbedtls_stub/altcom_mbedtls_hdlr.h
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __MODULES_MBEDTLS_STUBS_MBEDTLS_HDLR_H
#define __MODULES_MBEDTLS_STUBS_MBEDTLS_HDLR_H

 /****************************************************************************
 * Included Files
 ****************************************************************************/

#include "apicmd_ssl_init.h"
#include "apicmd_ssl.h"
#include "apicmd_ssl_free.h"
#include "apicmd_ssl_setup.h"
#include "apicmd_ssl_hostname.h"
#include "apicmd_ssl_bio.h"
#include "apicmd_ssl_handshake.h"
#include "apicmd_ssl_write.h"
#include "apicmd_ssl_read.h"
#include "apicmd_ssl_close_notify.h"
#include "apicmd_ssl_version.h"
#include "apicmd_ssl_ciphersuite.h"
#include "apicmd_ssl_ciphersuite_id.h"
#include "apicmd_ssl_record_expansion.h"
#include "apicmd_ssl_verify_result.h"
#include "apicmd_ssl_timer_cb.h"
#include "apicmd_ssl_peer_cert.h"
#include "apicmd_ssl_bytes_avail.h"
#include "apicmd_config_init.h"
#include "apicmd_config.h"
#include "apicmd_config_free.h"
#include "apicmd_config_defaults.h"
#include "apicmd_config_authmode.h"
#include "apicmd_config_rng.h"
#include "apicmd_config_ca_chain.h"
#include "apicmd_config_own_cert.h"
#include "apicmd_config_read_timeout.h"
#include "apicmd_config_verify.h"
#include "apicmd_config_alpn_protocols.h"
#include "apicmd_config_ciphersuites.h"
#include "apicmd_session_init.h"
#include "apicmd_session.h"
#include "apicmd_session_free.h"
#include "apicmd_session_get.h"
#include "apicmd_session_set.h"
#include "apicmd_x509_crt_init.h"
#include "apicmd_x509_crt.h"
#include "apicmd_x509_crt_info.h"
#include "apicmd_x509_crt_free.h"
#include "apicmd_x509_crt_parse_file.h"
#include "apicmd_x509_crt_parse_der.h"
#include "apicmd_x509_crt_parse.h"
#include "apicmd_x509_crt_verify_info.h"
#include "apicmd_pk_init.h"
#include "apicmd_pk.h"
#include "apicmd_pk_free.h"
#include "apicmd_pk_parse_keyfile.h"
#include "apicmd_pk_parse_key.h"
#include "apicmd_pk_check_pair.h"
#include "apicmd_pk_setup.h"
#include "apicmd_pk_info_from_type.h"
#include "apicmd_pk_write_key_pem.h"
#include "apicmd_pk_write_key_der.h"
#include "apicmd_pk_rsa.h"
#include "apicmd_ctr_drbg_init.h"
#include "apicmd_ctr_drbg.h"
#include "apicmd_ctr_drbg_free.h"
#include "apicmd_ctr_drbg_seed.h"
#include "apicmd_entropy_init.h"
#include "apicmd_entropy.h"
#include "apicmd_entropy_free.h"
#include "apicmd_cipher_init.h"
#include "apicmd_cipher_free.h"
#include "apicmd_cipher_info_from_string.h"
#include "apicmd_cipher_setup.h"
#include "apicmd_cipher_setkey.h"
#include "apicmd_cipher_set_iv.h"
#include "apicmd_cipher_update.h"
#include "apicmd_md_info_from_type.h"
#include "apicmd_cipher.h"
#include "apicmd_md_get_size.h"
#include "apicmd_md.h"
#include "apicmd_md_digest.h"
#include "apicmd_base64_encode.h"
#include "apicmd_sha1.h"
#include "apicmd_ssl_export_srtp_keys.h"
#include "apicmd_ssl_use_srtp.h"
#include "apicmd_ssl_srtp_profile.h"
#include "apicmd_ssl_turn.h"
#include "apicmd_mpi_init.h"
#include "apicmd_mpi_free.h"
#include "apicmd_mpi_read_string.h"
#include "apicmd_mpi_write_string.h"
#include "apicmd_x509_csr_init.h"
#include "apicmd_x509_csr_free.h"
#include "apicmd_x509_csr_parse_file.h"
#include "apicmd_x509_csr_parse_der.h"
#include "apicmd_x509_csr_parse.h"
#include "apicmd_x509_dn_gets_crt.h"
#include "apicmd_x509_dn_gets_csr.h"
#include "apicmd_x509write_crt_init.h"
#include "apicmd_x509write_crt_free.h"
#include "apicmd_x509write_crt_der.h"
#include "apicmd_x509write_crt_pem.h"
#include "apicmd_x509write_crt_subject_key.h"
#include "apicmd_x509write_crt_issuer_key.h"
#include "apicmd_x509write_crt_subject_name.h"
#include "apicmd_x509write_crt_issuer_name.h"
#include "apicmd_x509write_crt_version.h"
#include "apicmd_x509write_crt_md_alg.h"
#include "apicmd_x509write_crt_serial.h"
#include "apicmd_x509write_crt_validity.h"
#include "apicmd_x509write_crt_basic_constraints.h"
#include "apicmd_x509write_crt_subject_key_identifier.h"
#include "apicmd_x509write_crt_authority_key_identifier.h"
#include "apicmd_x509write_crt_key_usage.h"
#include "apicmd_x509write_crt_ns_cert_type.h"
#include "apicmd_rsa_init.h"
#include "apicmd_rsa_free.h"
#include "apicmd_rsa_gen_key.h"
#include "apicmd_config_verify_callback.h"

#endif /* __MODULES_MBEDTLS_STUBS_MBEDTLS_HDLR_H */
