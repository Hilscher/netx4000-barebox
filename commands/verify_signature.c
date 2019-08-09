#include <common.h>
#include <command.h>
#include <getopt.h>
#include <errno.h>
#include <rsa.h>
#include <fs.h>
#include <libfile.h>

#define DEFAULT_KEY_NODE_NAME  "/signature/key"

static int do_verify_signature(int argc, char *argv[])
{
	int ret, opt, verbose = 0;
	char *file = NULL, *sig_file = NULL, *dt_file = NULL, *key_node_name = DEFAULT_KEY_NODE_NAME;
	void *file_data, *sig_file_data, *dt_file_data;
	size_t file_data_len, sig_file_data_len, dt_file_data_len;
	char *file_hash = NULL;
	enum hash_algo algo = HASH_ALGO_SHA512;
	struct device_node *root_node = NULL, *key_node = NULL;
	struct rsa_public_key key;
	struct digest *digest = NULL;


	/* Retrieve optional arguments */
	while ((opt = getopt(argc, argv, "a:d:k:s:v")) > 0) {
		switch(opt) {
			case 'a':
				if(strcmp(optarg, "sha1") == 0)
					algo = HASH_ALGO_SHA1;
				else if(strcmp(optarg, "sha256") == 0)
					algo = HASH_ALGO_SHA256;
				else if(strcmp(optarg, "sha512") == 0)
					algo = HASH_ALGO_SHA512;
				else {
					pr_err("Invalid algo %s\n", optarg);
					return COMMAND_ERROR_USAGE;
				}
				break;
			case 'd':
				dt_file = optarg;
				break;
			case 'k':
				key_node_name = optarg;
				break;
			case 's':
				sig_file = optarg;
				break;
			case 'v':
				verbose++;
				break;
			default:
				return COMMAND_ERROR_USAGE;
		}
	}

	/* Retrieve mandatory arguments */
	if (argc - optind < 1)
		return COMMAND_ERROR_USAGE;
	file = argv[optind++];

	/* Read file data */
	ret = read_file_2(file, &file_data_len, &file_data, FILESIZE_MAX);
	if (ret < 0) {
		pr_err("Error reading file %s.\n", file);
		goto err_out;
	}

	/* Read key data */
	if (dt_file) {
		if (verbose)
			pr_info("Using device-tree file %s.\n", dt_file);

		ret = read_file_2(dt_file, &dt_file_data_len, &dt_file_data, FILESIZE_MAX);
		if (ret < 0) {
			pr_err("Error reading device-tree file %s.\n", dt_file);
			goto err_out;
		}

		root_node = of_unflatten_dtb(dt_file_data);
		if(IS_ERR(root_node)) {
			pr_err("Invalid or missing root_node in %s.\n", dt_file);
			return PTR_ERR(root_node);
		}
	}

	if (verbose)
		pr_info("Using key node %s from %s.\n", key_node_name, (dt_file) ? dt_file : "built-in device-tree");

	key_node = of_find_node_by_path_from(root_node, key_node_name);
	if(!key_node) {
		pr_err("Invalid or missing node %s.\n", key_node_name);
		ret = -EINVAL;
		goto err_out;
	}

	if((ret = rsa_of_read_key(key_node, &key))) {
		pr_err("Error reading key %s\n", key_node->full_name);
		ret = -EINVAL;
		goto err_out;
	}

	/* Read signature data */
	if (!sig_file) {
		sig_file = xzalloc(strlen(file)+sizeof(".sig"));
		BUG_ON(!sig_file);
		sprintf(sig_file, "%s.sig", file);
	}

	if (verbose)
		pr_info("Using signature file %s.\n", sig_file);

	ret = read_file_2(sig_file, &sig_file_data_len, &sig_file_data, FILESIZE_MAX);
	if (ret < 0) {
		pr_err("Error reading signature file %s.\n", sig_file);
		goto err_out;
	}

	/* Calculating file hash */
	digest = digest_alloc_by_algo(algo);
	BUG_ON(!digest);

	file_hash = xzalloc(digest_length(digest));
	BUG_ON(!file_hash);

	digest_init(digest);
	digest_update(digest, file_data, file_data_len);
	digest_final(digest, file_hash);

	/* Verify file signature */
	ret = rsa_verify(&key, sig_file_data, sig_file_data_len, file_hash, algo);
	if (ret) {
		pr_info("%s: Signature BAD\n", file);
		ret = -EBADMSG;
	}
	else {
		pr_info("%s: Signature OK\n", file);
		ret = 0;
	}

err_out:
	if(!file_data)
		free(file_data);
	if(!sig_file_data)
		free(sig_file_data);
	if(!dt_file_data)
		free(dt_file_data);
	if(!digest)
		digest_free(digest);
	if(!file_hash)
		free(file_hash);
	if(!root_node)
		of_delete_node(root_node);

	return ret;
}

BAREBOX_CMD_HELP_START(verify_signature)
BAREBOX_CMD_HELP_TEXT("Options:")
BAREBOX_CMD_HELP_OPT ("-a ALGO", "use algorithmus sha1, sha256 or sha512 (default: sha512)")
BAREBOX_CMD_HELP_OPT ("-d FILE", "use device-tree file as key storage (default: built-in)")
BAREBOX_CMD_HELP_OPT ("-k NODE", "use key node (default: "DEFAULT_KEY_NODE_NAME")")
BAREBOX_CMD_HELP_OPT ("-s FILE", "use signature file (default: $file.sig)")
BAREBOX_CMD_HELP_OPT ("-v", "increase verbose level")
BAREBOX_CMD_HELP_END

BAREBOX_CMD_START(verify_signature)
	.cmd = do_verify_signature,
	BAREBOX_CMD_DESC("Verify signature against known public key.")
	BAREBOX_CMD_OPTS("[-aks] file")
	BAREBOX_CMD_GROUP(CMD_GRP_FILE)
	BAREBOX_CMD_HELP(cmd_verify_signature_help)
BAREBOX_CMD_END
