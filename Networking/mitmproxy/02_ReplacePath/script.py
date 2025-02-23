
import logging

from mitmproxy import ctx

class Replacer:
    def load(self, loader):
        loader.add_option(
            name="domain",
            typespec=str,
            default="",
            help="target domain",
        )
        loader.add_option(
            name="old",
            typespec=str,
            default="",
            help="old path to be replaced",
        )
        loader.add_option(
            name="new",
            typespec=str,
            default="",
            help="new path",
        )

    def request(self, flow):
        domain = ctx.options.domain
        old_path = ctx.options.old
        new_path = ctx.options.new
        
        # Note: flow.request.url is ip address. why?
        index = flow.client_conn.sni.find(domain)
        if index == -1:
            return
        if not flow.request.path == old_path:
            return
    
        flow.request.path = new_path

addons = [Replacer()]